#include "motor.h"

#include <math.h>

#include "devices/hw/encoder.h"
#include "devices/io/can_comm.h"
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
        constexpr float alignment_voltage = 0.4f;
        constexpr uint32_t alignment_duration_ms = 500;
        constexpr uint32_t encoder_wait_timeout_ms = 100;
        constexpr uint16_t direction_steps = 100;
        constexpr uint32_t direction_step_duration_ms = 2;
        constexpr float direction_min_delta = two_pi / 100.0f;
        constexpr uint32_t target_timeout_ms = 100;
        // TIM2 CH4 的 CCR4=800 比较事件只在向下计数时产生：20 kHz / 2 = 10 kHz。
        constexpr uint8_t control_update_divider = 2;

        volatile bool initialized = false;
        volatile uint8_t update_count = 0;
        float rotor_direction = 0.0f;
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
        void set_phase_voltage(float uq, float electrical_angle)
        {
            if(!(uq == uq)){uq = 0.0f;}
            if(uq > voltage_limit){uq = voltage_limit;}
            else if(uq < -voltage_limit){uq = -voltage_limit;}

            if(uq > bus_voltage * max_modulation)
            {
                uq = bus_voltage * max_modulation;
            }
            else if(uq < -bus_voltage * max_modulation)
            {
                uq = -bus_voltage * max_modulation;
            }

            float sine;
            float cosine;
            foc_sincos(normalize_angle(electrical_angle), &sine, &cosine);

            // Inverse Park transform with Ud fixed to zero, followed by Clarke.
            const float alpha = -sine * uq;
            const float beta = cosine * uq;
            float phase_a = alpha;
            float phase_b = -0.5f * alpha + sqrt_three_half * beta;
            float phase_c = -0.5f * alpha - sqrt_three_half * beta;

            // SimpleFOC SpaceVectorPWM 的居中零序注入。
            float phase_min = phase_a;
            float phase_max = phase_a;
            if(phase_b < phase_min){phase_min = phase_b;}
            if(phase_c < phase_min){phase_min = phase_c;}
            if(phase_b > phase_max){phase_max = phase_b;}
            if(phase_c > phase_max){phase_max = phase_c;}

            const float center = bus_voltage * 0.5f -
                (phase_max + phase_min) * 0.5f;
            phase_a = (phase_a + center) / bus_voltage;
            phase_b = (phase_b + center) / bus_voltage;
            phase_c = (phase_c + center) / bus_voltage;

            if(!(phase_a >= 0.0f)){phase_a = 0.0f;}
            else if(phase_a > 1.0f){phase_a = 1.0f;}
            if(!(phase_b >= 0.0f)){phase_b = 0.0f;}
            else if(phase_b > 1.0f){phase_b = 1.0f;}
            if(!(phase_c >= 0.0f)){phase_c = 0.0f;}
            else if(phase_c > 1.0f){phase_c = 1.0f;}

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                (uint32_t)(phase_a * 1600.0f + 0.5f));
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                (uint32_t)(phase_b * 1600.0f + 0.5f));
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                (uint32_t)(phase_c * 1600.0f + 0.5f));
        }

        void enable_driver()
        {
            HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
        }

        void disable_driver()
        {
            HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
        }

        void loop()
        {
            if(!initialized)
            {
                return;
            }

            encoder_package encoder = {};
            target_package target;
            float target_voltage = 0.0f;

            if(!as5600::get_package(encoder))
            {
                set_phase_voltage(0.0f, 0.0f);
                return;
            }

            if(can_comm::get_package(target) &&
               (uint32_t)(sys_time::get_ms_tick() - target.timestamp_ms) <=
                   target_timeout_ms)
            {
                target_voltage = target.voltage;
            }

            const float electrical_angle = normalize_angle(
                rotor_direction * (float)pole_pairs * encoder.angle -
                zero_electric_angle);
            set_phase_voltage(target_voltage, electrical_angle);
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

        if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
           HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
           HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
        {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
            return false;
        }

        set_phase_voltage(0.0f, 0.0f);

        encoder_package package = {};
        bool package_valid = false;
        const uint32_t encoder_start = sys_time::get_ms_tick();
        do
        {
            as5600::update();
            if(as5600::get_package(package))
            {
                package_valid = true;
                break;
            }
            sys_time::delay_ms(1);
        }
        while((uint32_t)(sys_time::get_ms_tick() - encoder_start) <
            encoder_wait_timeout_ms);

        if(!package_valid)
        {
            disable_driver();
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
            return false;
        }

        enable_driver();
        const float direction_start = package.full_angle;
        for(uint16_t step = 0; step <= direction_steps; ++step)
        {
            set_phase_voltage(
                alignment_voltage,
                three_pi_2 + two_pi * (float)step / (float)direction_steps);
            as5600::update();
            sys_time::delay_ms(direction_step_duration_ms);
        }

        package_valid = as5600::get_package(package);
        if(!package_valid)
        {
            set_phase_voltage(0.0f, 0.0f);
            disable_driver();
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
            return false;
        }

        const float direction_delta = package.full_angle - direction_start;
        if(direction_delta > direction_min_delta)
        {
            rotor_direction = 1.0f;
        }
        else if(direction_delta < -direction_min_delta)
        {
            rotor_direction = -1.0f;
        }
        else
        {
            set_phase_voltage(0.0f, 0.0f);
            disable_driver();
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
            return false;
        }

        set_phase_voltage(0.0f, 0.0f);
        disable_driver();

        set_phase_voltage(alignment_voltage, three_pi_2);
        enable_driver();
        const uint32_t alignment_start = sys_time::get_ms_tick();
        while((uint32_t)(sys_time::get_ms_tick() - alignment_start) <
            alignment_duration_ms)
        {
            as5600::update();
            sys_time::delay_ms(1);
        }

        package_valid = as5600::get_package(package);
        set_phase_voltage(0.0f, 0.0f);
        disable_driver();
        if(!package_valid)
        {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
            return false;
        }

        zero_electric_angle = normalize_angle(
            rotor_direction * (float)pole_pairs * package.angle);

        enable_driver();
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);
        update_count = 0;
        initialized = true;
        if(HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
        {
            initialized = false;
            disable_driver();
            HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
            return false;
        }

        return true;
    }

}

extern "C" void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *timer)
{
    if(timer != &htim2 || timer->Channel != HAL_TIM_ACTIVE_CHANNEL_4)
    {
        return;
    }

    motor::update_count++;
    if(motor::update_count >= motor::control_update_divider)
    {
        motor::update_count = 0;
        motor::loop();
    }
}
