#include "motor.h"

#include "devices/hw/encoder.h"
#include "devices/hw/fixed_point.h"
#include "devices/io/can_comm.h"
#include "main.h"
#include "sys_time.h"
#include "tim.h"

namespace motor
{
    namespace
    {
        using fixed::q15_t;
        using fixed::q16_t;

        // 相位使用一圈映射到 0x0000...0xFFFF，天然支持取模。
        constexpr uint16_t phase_quarter_turn = 0x4000u;
        constexpr uint16_t phase_three_quarter_turn = 0xC000u;

        // 电机 / 驱动固定参数，物理量使用 Q16.16。
        constexpr uint8_t pole_pairs = 7;
        constexpr q16_t phase_resistance_q16 = 167117;          // 2.55 Ω
        constexpr q16_t inverse_torque_constant_q16 = 1510046; // 1 / 0.0434 A/Nm
        constexpr q16_t bemf_constant_q16 = 1642;               // 0.02506 V/(rad/s)
        constexpr q16_t bus_voltage_inv_q16 = 5461;             // 1 / 12 V

        // SVPWM 最大可用 q 轴电压约为 Vbus / sqrt(3)。
        constexpr q16_t output_limit_q16 = 454047;              // 6.9282 V
        constexpr q16_t alignment_voltage_q16 = 196608;         // 3.0 V

        // Q15 常量：1.0 = 32768。
        constexpr q15_t q15_half = 16384;
        constexpr q15_t sqrt_three_half_q15 = 28378;

        constexpr uint16_t pwm_period = 1600;
        constexpr uint16_t pwm_midpoint = pwm_period / 2;

        // AS5600 一个原始计数约为 0.001534 rad。
        constexpr int32_t direction_min_delta_counts = 41;
        constexpr uint32_t encoder_timeout_ms = 5;

        int8_t direction = 0;
        uint16_t zero_phase = 0;
        bool initialized = false;

        /**
         * @brief 正弦查表，返回 Q15。
         *
         * phase:
         * 0x0000 -> 0
         * 0x4000 -> π/2
         * 0x8000 -> π
         * 0xC000 -> 3π/2
         */
        q15_t lookup_sin(uint16_t phase)
        {
            static const uint16_t table[65] =
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

            const uint16_t index = phase >> 8;
            const int32_t fraction = phase & 0xFF;

            int32_t a;
            int32_t b;

            if(index < 64)
            {
                a = table[index];
                b = table[index + 1];
            }
            else if(index < 128)
            {
                a = table[128 - index];
                b = table[127 - index];
            }
            else if(index < 192)
            {
                a = -table[index - 128];
                b = -table[index - 127];
            }
            else
            {
                a = -table[256 - index];
                b = -table[255 - index];
            }

            return static_cast<q15_t>(
                a + (((b - a) * fraction) >> 8)
            );
        }

        /**
         * @brief 同时计算 Q15 正弦和余弦。
         */
        void sin_cos(uint16_t phase, q15_t &sine, q15_t &cosine)
        {
            sine = lookup_sin(phase);
            cosine = lookup_sin(
                static_cast<uint16_t>(phase + phase_quarter_turn)
            );
        }

        q16_t clamp_voltage(q16_t voltage)
        {
            if(voltage > output_limit_q16){return output_limit_q16;}
            if(voltage < -output_limit_q16){return -output_limit_q16;}
            return voltage;
        }

        q15_t voltage_to_q15(q16_t voltage)
        {
            // voltage / Vbus 先得到 Q16.16，再换算为 Q15。
            return fixed::mul_q16(voltage, bus_voltage_inv_q16) >> 1;
        }

        q15_t clamp_duty(q15_t duty)
        {
            if(duty < 0){return 0;}
            if(duty > fixed::q15_one){return fixed::q15_one;}
            return duty;
        }

        uint16_t duty_to_ccr(q15_t duty)
        {
            duty = clamp_duty(duty);

            return static_cast<uint16_t>(
                (static_cast<int64_t>(duty) * pwm_period + q15_half) >> 15
            );
        }

        void write_midpoint()
        {
            TIM2->CCR1 = pwm_midpoint;
            TIM2->CCR2 = pwm_midpoint;
            TIM2->CCR3 = pwm_midpoint;
        }

        /**
         * @brief 关闭驱动、PWM 和 FOC 中断。
         *
         * 该路径也由编码器超时故障触发，先把比较值置中再关桥。
         */
        void stop_output()
        {
            initialized = false;
            write_midpoint();

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET
            );

            __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC4);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
        }

        /**
         * @brief SpaceVectorPWM 核心路径。
         *
         * voltage 为 Q16.16 q 轴目标电压，d 轴固定为 0；angle 为一圈相位。
         * 内部完成逆 Park、逆 Clarke、零序注入和 TIM2 比较值更新。
         */
        void svpwm(q16_t voltage, uint16_t phase)
        {
            voltage = clamp_voltage(voltage);

            q15_t sine;
            q15_t cosine;
            sin_cos(phase, sine, cosine);

            const q15_t uq = voltage_to_q15(voltage);

            // Ud = 0：逆 Park。
            const q15_t alpha = -fixed::mul_q15(sine, uq);
            const q15_t beta = fixed::mul_q15(cosine, uq);

            // 逆 Clarke。
            q15_t phase_a = alpha;
            q15_t phase_b =
                -fixed::mul_q15(alpha, q15_half) +
                fixed::mul_q15(sqrt_three_half_q15, beta);
            q15_t phase_c =
                -fixed::mul_q15(alpha, q15_half) -
                fixed::mul_q15(sqrt_three_half_q15, beta);

            // SVPWM 零序注入：0.5 - 0.5 * (max + min)。
            q15_t phase_min = phase_a;
            q15_t phase_max = phase_a;

            if(phase_b < phase_min){phase_min = phase_b;}
            if(phase_c < phase_min){phase_min = phase_c;}

            if(phase_b > phase_max){phase_max = phase_b;}
            if(phase_c > phase_max){phase_max = phase_c;}

            const q15_t offset =
                q15_half -
                fixed::mul_q15(phase_max + phase_min, q15_half);

            phase_a += offset;
            phase_b += offset;
            phase_c += offset;

            TIM2->CCR1 = duty_to_ccr(phase_a);
            TIM2->CCR2 = duty_to_ccr(phase_b);
            TIM2->CCR3 = duty_to_ccr(phase_c);
        }

        bool encoder_is_fresh(const encoder_package &encoder)
        {
            return sys_time::get_ms_tick() - encoder.timestamp_ms <=
                encoder_timeout_ms;
        }

        /**
         * @brief 自动检测编码器方向并校准零电角。
         */
        bool calibrate()
        {
            encoder_package encoder = {};

            // 等待编码器第一帧数据。
            const uint32_t start = sys_time::get_ms_tick();
            while(sys_time::get_ms_tick() - start < 100)
            {
                as5600::update();
                if(as5600::get_package(encoder) && encoder.sequence != 0)
                {
                    break;
                }
                sys_time::delay_ms(1);
            }

            if(!as5600::get_package(encoder) || encoder.sequence == 0)
            {
                return false;
            }

            // 检测编码器方向。
            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_SET
            );

            const int32_t start_count = encoder.full_count;

            for(uint16_t i = 0; i <= 100; ++i)
            {
                const uint16_t phase = static_cast<uint16_t>(
                    phase_three_quarter_turn +
                    (static_cast<uint32_t>(i) * 65536u) / 100u
                );

                svpwm(alignment_voltage_q16, phase);
                as5600::update();
                sys_time::delay_ms(2);
            }

            if(!as5600::get_package(encoder))
            {
                return false;
            }

            const int32_t delta_count = encoder.full_count - start_count;
            if(delta_count > direction_min_delta_counts)
            {
                direction = 1;
            }
            else if(delta_count < -direction_min_delta_counts)
            {
                direction = -1;
            }
            else
            {
                return false;
            }

            // 电角度对齐。
            svpwm(0, 0);

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET
            );

            svpwm(alignment_voltage_q16, phase_three_quarter_turn);

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_SET
            );

            const uint32_t align_start = sys_time::get_ms_tick();
            while(sys_time::get_ms_tick() - align_start < 500)
            {
                as5600::update();
                sys_time::delay_ms(1);
            }

            if(!as5600::get_package(encoder))
            {
                return false;
            }

            zero_phase = static_cast<uint16_t>(
                static_cast<int32_t>(direction) *
                pole_pairs *
                static_cast<int32_t>(encoder.angle_phase)
            );

            svpwm(0, 0);

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET
            );

            return true;
        }

        /**
         * @brief FOC 实时更新，由 TIM2 CH4 中断周期调用。
         */
        void update()
        {
            if(!initialized)
            {
                return;
            }

            encoder_package encoder = {};
            if(!as5600::get_package(encoder) || !encoder_is_fresh(encoder))
            {
                // 编码器停止更新时禁止继续使用旧角度，避免定子锁死发热。
                stop_output();
                return;
            }

            const int32_t electrical_phase =
                static_cast<int32_t>(direction) *
                pole_pairs *
                static_cast<int32_t>(encoder.angle_phase) -
                static_cast<int32_t>(zero_phase);

            const q16_t torque_target = can_comm::get_target_q16();

            // Nm -> q 轴电流，再计算电阻压降和反电势补偿。
            const q16_t current = fixed::mul_q16(
                torque_target,
                inverse_torque_constant_q16
            );

            const q16_t velocity = direction >= 0
                ? encoder.speed_q16
                : fixed::saturate_q16(-static_cast<int64_t>(encoder.speed_q16));

            const q16_t bemf = fixed::mul_q16(
                bemf_constant_q16,
                velocity
            );

            const q16_t resistive_voltage = fixed::mul_q16(
                current,
                phase_resistance_q16
            );

            svpwm(
                fixed::add_q16(resistive_voltage, bemf),
                static_cast<uint16_t>(electrical_phase)
            );
        }
    }

    /**
     * @brief 初始化电机 FOC。
     */
    bool init()
    {
        initialized = false;

        // 保证启动阶段驱动关闭。
        HAL_GPIO_WritePin(
            DRV_EN_GPIO_Port,
            DRV_EN_Pin,
            GPIO_PIN_RESET
        );

        // 防止 TIM2 初始化阶段残留中断状态。
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC4);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);

        // 启动三相 PWM。
        if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
           HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
           HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
        {
            stop_output();
            return false;
        }

        svpwm(0, 0);

        // 编码器方向和零电角自动校准。
        if(!calibrate())
        {
            stop_output();
            return false;
        }

        // 校准完成，开启驱动。
        HAL_GPIO_WritePin(
            DRV_EN_GPIO_Port,
            DRV_EN_Pin,
            GPIO_PIN_SET
        );

        // CH4 位于 PWM 中点，用于触发 FOC。
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);

        if(HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
        {
            stop_output();
            return false;
        }

        initialized = true;
        return true;
    }
}

/**
 * @brief TIM2 CH4 FOC 中断。
 *
 * Center-aligned mode 1 下 CH4 每个 PWM 周期触发一次，这里再二分。
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *timer)
{
    if(timer != &htim2 ||
       timer->Channel != HAL_TIM_ACTIVE_CHANNEL_4)
    {
        return;
    }

    static bool toggle = false;
    toggle = !toggle;
    if(toggle){return;}
    motor::update();
}
