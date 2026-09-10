#include "motor.h"

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

        // 当前电机硬件参数：2804，14 极，编码器方向为正。
        constexpr uint8_t pole_pairs = 7;
        constexpr float rotor_direction = 1.0f;
        constexpr float bus_voltage = 12.0f;
        constexpr float voltage_limit = 4.0f;
        constexpr float alignment_voltage = 0.4f;
        constexpr uint32_t alignment_duration_ms = 500;
        constexpr uint32_t encoder_wait_timeout_ms = 100;
        constexpr uint32_t target_timeout_ms = 100;
        // TIM2 中心对齐下每个 PWM 周期产生两次更新事件：40 kHz / 4 = 10 kHz。
        constexpr uint8_t control_update_divider = 4;

        volatile bool initialized = false;
        volatile uint8_t update_count = 0;
        float zero_electric_angle = 0.0f;

        /**
         * @brief 将角度限制到 [0, 2π)
         */
        float normalize_angle(float angle)
        {
            while(angle >= two_pi){angle -= two_pi;}
            while(angle < 0.0f){angle += two_pi;}
            return angle;
        }

        float sine_approx(float angle)
        {
            const float angle_squared = angle * angle;
            return angle * (1.0f + angle_squared *
                (-0.1666666667f + angle_squared *
                (0.0083333333f + angle_squared *
                (-0.0001984127f + angle_squared *
                (0.0000027557f - angle_squared * 0.0000000251f)))));
        }

        /**
         * @brief 计算当前电角度的正弦和余弦
         *
         * 使用四象限映射和短 Taylor 近似，避免在 F103 的控制中断中引入
         * 完整 sinf/cosf 路径。
         */
        void sine_cosine(float angle, float &sine, float &cosine)
        {
            angle = normalize_angle(angle);

            const uint8_t quadrant = (uint8_t)(angle / pi_2);
            const float offset = angle - (float)quadrant * pi_2;
            const float sine_offset = sine_approx(offset);
            const float cosine_offset = sine_approx(pi_2 - offset);

            switch(quadrant)
            {
                case 0:
                    sine = sine_offset;
                    cosine = cosine_offset;
                    break;

                case 1:
                    sine = cosine_offset;
                    cosine = -sine_offset;
                    break;

                case 2:
                    sine = -sine_offset;
                    cosine = -cosine_offset;
                    break;

                default:
                    sine = -cosine_offset;
                    cosine = sine_offset;
                    break;
            }
        }

        float limit_duty(float duty)
        {
            if(!(duty >= 0.0f)){return 0.0f;}
            return duty > 1.0f ? 1.0f : duty;
        }

        uint32_t duty_to_compare(float duty)
        {
            const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2);
            return (uint32_t)(limit_duty(duty) * (float)period + 0.5f);
        }

        /**
         * @brief 向当前固定的 TIM2 三相输出写入占空比
         *
         * 逻辑 A/B/C 直接对应 TIM2 CH1/CH2/CH3（PA0/PA1/PA2）。
         */
        void write_duty(float phase_a, float phase_b, float phase_c)
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                duty_to_compare(phase_a));
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                duty_to_compare(phase_b));
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                duty_to_compare(phase_c));
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

            const float max_voltage = bus_voltage * max_modulation;
            if(uq > max_voltage){uq = max_voltage;}
            else if(uq < -max_voltage){uq = -max_voltage;}

            float sine;
            float cosine;
            sine_cosine(electrical_angle, sine, cosine);

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

            write_duty(phase_a, phase_b, phase_c);
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
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
        }

        bool start_pwm()
        {
            if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
            {
                return false;
            }
            if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
            {
                stop_pwm();
                return false;
            }
            if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
            {
                stop_pwm();
                return false;
            }
            return true;
        }

        bool read_encoder(encoder_package &package)
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
            while((uint32_t)(sys_time::get_ms_tick() - start) <
                encoder_wait_timeout_ms);

            return false;
        }

        void update_encoder(uint32_t duration_ms)
        {
            const uint32_t start = sys_time::get_ms_tick();

            while((uint32_t)(sys_time::get_ms_tick() - start) < duration_ms)
            {
                as5600::update();
                sys_time::delay_ms(1);
            }
        }

        /**
         * @brief 用固定方向电压矢量完成一次硬件专用编码器对齐
         */
        bool align_sensor()
        {
            encoder_package package;
            if(!read_encoder(package))
            {
                return false;
            }

            set_phase_voltage(alignment_voltage, three_pi_2);
            enable_driver();
            update_encoder(alignment_duration_ms);

            const bool package_valid = as5600::get_package(package);
            set_phase_voltage(0.0f, 0.0f);
            disable_driver();

            if(!package_valid)
            {
                return false;
            }

            zero_electric_angle = normalize_angle(
                rotor_direction * (float)pole_pairs * package.angle);
            return true;
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
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

        if(!start_pwm())
        {
            return false;
        }

        set_phase_voltage(0.0f, 0.0f);
        if(!align_sensor())
        {
            set_phase_voltage(0.0f, 0.0f);
            disable_driver();
            stop_pwm();
            return false;
        }

        set_phase_voltage(0.0f, 0.0f);
        enable_driver();
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        update_count = 0;
        initialized = true;
        __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

        return true;
    }

    static void on_timer_elapsed(TIM_HandleTypeDef *timer)
    {
        if(timer != &htim2)
        {
            return;
        }

        update_count++;
        if(update_count >= control_update_divider)
        {
            update_count = 0;
            loop();
        }
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    motor::on_timer_elapsed(timer);
}
