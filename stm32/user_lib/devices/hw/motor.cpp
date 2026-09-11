#include "motor.h"

#include <math.h>

#include "devices/hw/encoder.h"
#include "main.h"
#include "sys_time.h"
#include "tim.h"

namespace motor
{
    namespace
    {
        constexpr float pi_2 = 1.57079632679489661923f;
        constexpr float three_pi_2 = 4.71238898038468985769f;
        constexpr float two_pi = 6.28318530717958647692f;
        constexpr float sqrt_three_half = 0.86602540378443864676f;
        constexpr float max_modulation = 0.57735026918962576451f;

        // 当前电机硬件参数：2804，14 极；编码器方向在初始化时自动判断。
        constexpr uint8_t pole_pairs = 7;
        constexpr float bus_voltage = 12.0f;
        constexpr float voltage_limit = 4.0f;
        constexpr float svpwm_voltage_limit = bus_voltage * max_modulation;
        constexpr float effective_voltage_limit =
            voltage_limit < svpwm_voltage_limit
                ? voltage_limit
                : svpwm_voltage_limit;
        constexpr float bus_voltage_inv = 1.0f / bus_voltage;
        constexpr float pwm_period = 1600.0f;
        constexpr float alignment_voltage = 0.4f;
        constexpr uint16_t direction_steps = 100;
        constexpr float direction_min_delta = two_pi / 100.0f;

        volatile bool initialized = false;
        int8_t rotor_direction = 0;
        float zero_electric_angle = 0.0f;

        // SimpleFOC 的角度归一化实现。
        float normalize_angle(float angle)
        {
            const float normalized = fmodf(angle, two_pi);
            return normalized >= 0.0f ? normalized : normalized + two_pi;
        }

        // SimpleFOC 的查表插值实现，保留 _sin/_cos/_sincos 的计算路径。
        float foc_sin(float angle)
        {
            static const uint16_t sine_array[65] =
            {
                0, 804, 1608, 2411, 3212, 4011, 4808, 5602,
                6393, 7180, 7962, 8740, 9512, 10279, 11039, 11793,
                12540, 13279, 14010, 14733, 15447, 16151, 16846, 17531,
                18205, 18868, 19520, 20160, 20788, 21403, 22006, 22595,
                23170, 23732, 24279, 24812, 25330, 25833, 26320, 26791,
                27246, 27684, 28106, 28511, 28899, 29269, 29622, 29957,
                30274, 30572, 30853, 31114, 31357, 31581, 31786, 31972,
                32138, 32286, 32413, 32522, 32610, 32679, 32729, 32758,
                32768
            };

            int32_t t1;
            int32_t t2;
            unsigned int index =
                (unsigned int)(angle * (64U * 4U * 256.0f / two_pi));
            const int32_t fraction = (int32_t)(index & 0xffU);
            index = (index >> 8) & 0xffU;

            if(index < 64U)
            {
                t1 = sine_array[index];
                t2 = sine_array[index + 1U];
            }
            else if(index < 128U)
            {
                t1 = sine_array[128U - index];
                t2 = sine_array[127U - index];
            }
            else if(index < 192U)
            {
                t1 = -sine_array[index - 128U];
                t2 = -sine_array[index - 127U];
            }
            else
            {
                t1 = -sine_array[256U - index];
                t2 = -sine_array[255U - index];
            }

            return (1.0f / 32768.0f) *
                (t1 + (((t2 - t1) * fraction) >> 8));
        }

        float foc_cos(float angle)
        {
            float sine_angle = angle + pi_2;
            sine_angle = sine_angle > two_pi ? sine_angle - two_pi : sine_angle;
            return foc_sin(sine_angle);
        }

        void foc_sincos(float angle, float *sine, float *cosine)
        {
            *sine = foc_sin(angle);
            *cosine = foc_cos(angle);
        }

        /**
         * @brief SimpleFOC 电压力矩模式的 SpaceVectorPWM 核心路径
         *
         * 只保留 Ud=0：逆 Park、Clarke 和居中零序注入；Uq 单位为伏特，
         * 电角度单位为弧度。
         */
        void svpwm(float uq, float electrical_angle)
        {
            if(!(uq == uq)){uq = 0.0f;}
            if(uq > effective_voltage_limit){uq = effective_voltage_limit;}
            else if(uq < -effective_voltage_limit){uq = -effective_voltage_limit;}

            float sine;
            float cosine;
            foc_sincos(normalize_angle(electrical_angle), &sine, &cosine);

            // 逆 Park（Ud 固定为 0）和 Clarke 变换。
            const float alpha = -sine * uq;
            const float beta = cosine * uq;
            float phase_a = alpha;
            float phase_b = -0.5f * alpha + sqrt_three_half * beta;
            float phase_c = -0.5f * alpha - sqrt_three_half * beta;

            // 居中零序注入，保持 SimpleFOC 的 SpaceVectorPWM 路径。
            float phase_min = phase_a;
            float phase_max = phase_a;
            if(phase_b < phase_min){phase_min = phase_b;}
            if(phase_c < phase_min){phase_min = phase_c;}
            if(phase_b > phase_max){phase_max = phase_b;}
            if(phase_c > phase_max){phase_max = phase_c;}

            const float center = bus_voltage * 0.5f -
                (phase_max + phase_min) * 0.5f;
            phase_a = (phase_a + center) * bus_voltage_inv;
            phase_b = (phase_b + center) * bus_voltage_inv;
            phase_c = (phase_c + center) * bus_voltage_inv;

            if(!(phase_a >= 0.0f)){phase_a = 0.0f;}
            else if(phase_a > 1.0f){phase_a = 1.0f;}
            if(!(phase_b >= 0.0f)){phase_b = 0.0f;}
            else if(phase_b > 1.0f){phase_b = 1.0f;}
            if(!(phase_c >= 0.0f)){phase_c = 0.0f;}
            else if(phase_c > 1.0f){phase_c = 1.0f;}

            TIM2->CCR1 = (uint32_t)(phase_a * pwm_period + 0.5f);
            TIM2->CCR2 = (uint32_t)(phase_b * pwm_period + 0.5f);
            TIM2->CCR3 = (uint32_t)(phase_c * pwm_period + 0.5f);
        }

        void enable_driver()
        {
            HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
        }

        void disable_driver()
        {
            HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
        }

        void stop_pwm()
        {
            svpwm(0.0f, 0.0f);
            disable_driver();
            HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
        }

        bool start_pwm()
        {
            if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
               HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
               HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
            {
                stop_pwm();
                return false;
            }

            return true;
        }

        bool wait_encoder(encoder_package &package)
        {
            const uint32_t start = sys_time::get_ms_tick();
            do
            {
                as5600::update();
                if(as5600::get_package(package))
                {
                    return true;
                }
                sys_time::delay_ms(1);
            }
            while((uint32_t)(sys_time::get_ms_tick() - start) < 100);

            return false;
        }

        bool detect_direction(encoder_package &package)
        {
            enable_driver();
            const float direction_start = package.full_angle;
            for(uint16_t step = 0; step <= direction_steps; ++step)
            {
                svpwm(
                    alignment_voltage,
                    three_pi_2 + two_pi * (float)step /
                        (float)direction_steps);
                as5600::update();
                sys_time::delay_ms(2);
            }

            if(!as5600::get_package(package))
            {
                return false;
            }

            const float direction_delta = package.full_angle - direction_start;
            if(direction_delta > direction_min_delta)
            {
                rotor_direction = 1;
                return true;
            }
            if(direction_delta < -direction_min_delta)
            {
                rotor_direction = -1;
                return true;
            }

            return false;
        }

        bool align_encoder(encoder_package &package)
        {
            svpwm(0.0f, 0.0f);
            disable_driver();

            svpwm(alignment_voltage, three_pi_2);
            enable_driver();
            const uint32_t start = sys_time::get_ms_tick();
            while((uint32_t)(sys_time::get_ms_tick() - start) < 500)
            {
                as5600::update();
                sys_time::delay_ms(1);
            }

            const bool package_valid = as5600::get_package(package);
            svpwm(0.0f, 0.0f);
            disable_driver();
            if(!package_valid)
            {
                return false;
            }

            zero_electric_angle = normalize_angle(
                (float)rotor_direction * (float)pole_pairs * package.angle);
            return true;
        }

        void loop()
        {
            if(!initialized)
            {
                return;
            }

            encoder_package encoder = {};

            if(!as5600::get_package(encoder))
            {
                svpwm(0.0f, 0.0f);
                return;
            }

            // 临时固定目标，等待主控控制指令接入。
            const float target_voltage = 1.0f;

            const float electrical_angle =
                (float)rotor_direction * (float)pole_pairs * encoder.angle -
                zero_electric_angle;
            svpwm(target_voltage, electrical_angle);
        }
    }

    bool init()
    {
        if(initialized)
        {
            return true;
        }

        disable_driver();
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC4);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);

        if(!start_pwm())
        {
            return false;
        }

        svpwm(0.0f, 0.0f);

        encoder_package package = {};
        if(!wait_encoder(package))
        {
            stop_pwm();
            return false;
        }

        if(!detect_direction(package))
        {
            stop_pwm();
            return false;
        }

        if(!align_encoder(package))
        {
            stop_pwm();
            return false;
        }

        enable_driver();
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);
        if(HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
        {
            stop_pwm();
            return false;
        }

        initialized = true;
        return true;
    }

}

extern "C" void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *timer)
{
    static bool foc_toggle = false;

    if(timer != &htim2 || timer->Channel != HAL_TIM_ACTIVE_CHANNEL_4)
    {
        return;
    }

    foc_toggle = !foc_toggle;
    if(foc_toggle)
    {
        return;
    }

    motor::loop();
}
