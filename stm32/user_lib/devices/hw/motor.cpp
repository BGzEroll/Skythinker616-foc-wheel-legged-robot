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
        using q15_t = int32_t;
        using phase_t = uint16_t;

        constexpr q15_t Q15_ONE = 32768;
        constexpr q15_t Q15_HALF = 16384;

        // 电机参数
        constexpr int32_t pole_pairs = 7;

        // Q15 常量
        constexpr q15_t SQRT3_HALF = 28378;     // sqrt(3) / 2
        constexpr q15_t OUTPUT_LIMIT = 18919;   // 1 / sqrt(3)
        constexpr q15_t ALIGNMENT_UQ = 8192;    // 3V / 12V

        // 编码器校准参数
        constexpr uint16_t direction_steps = 100;
        constexpr int32_t direction_min_count = 41;

        /*
         * torque_mNm -> normalized Uq Q15
         *
         * R = 2.55 Ω
         * Kt = 0.0434 Nm/A
         * Vbus = 12 V
         */
        constexpr int32_t TORQUE_GAIN_Q8 = 41073;

        /*
         * speed_mrad_s -> normalized BEMF Q15
         *
         * Ke = 0.02506 V/(rad/s)
         * Vbus = 12 V
         */
        constexpr int32_t BEMF_GAIN_Q14 = 1121;

        int8_t direction = 0;
        phase_t zero_phase = 0;

        /**
         * @brief Q15 × Q15 -> Q15
         *
         * @param a Q15 输入
         * @param b Q15 输入
         *
         * @return Q15 乘法结果
         */
        inline q15_t mul_q15(q15_t a, q15_t b)
        {
            return (a * b) >> 15;
        }

        /**
         * @brief AS5600 count 转 16-bit phase
         *
         * 4096 count/rev -> 65536 phase/rev
         *
         * @param count AS5600 累计计数
         *
         * @return 16-bit 机械角 phase
         */
        inline phase_t get_phase(int32_t count)
        {
            return (phase_t)((uint32_t)count << 4);
        }

        /**
         * @brief Q15 正弦查表
         *
         * phase:
         * 0x0000 -> 0
         * 0x4000 -> π/2
         * 0x8000 -> π
         * 0xC000 -> 3π/2
         *
         * @param phase 16-bit 相位
         *
         * @return Q15 正弦值
         */
        q15_t lookup_sin(phase_t phase)
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

            int32_t a, b;

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

            return a + (((b - a) * fraction) >> 8);
        }

        /**
         * @brief Space Vector PWM
         *
         * @param uq q 轴归一化电压，Q15
         * @param phase 16-bit 电角度
         */
        void svpwm(q15_t uq, phase_t phase)
        {
            if(uq > OUTPUT_LIMIT)
            {
                uq = OUTPUT_LIMIT;
            }
            else if(uq < -OUTPUT_LIMIT)
            {
                uq = -OUTPUT_LIMIT;
            }

            const q15_t sine = lookup_sin(phase);
            const q15_t cosine = lookup_sin((phase_t)(phase + 0x4000));

            // Inverse Park
            const q15_t alpha = -mul_q15(sine, uq);
            const q15_t beta = mul_q15(cosine, uq);

            // Inverse Clarke
            q15_t phase_a = alpha;
            q15_t phase_b = -(alpha >> 1) + mul_q15(SQRT3_HALF, beta);
            q15_t phase_c = -(alpha >> 1) - mul_q15(SQRT3_HALF, beta);

            // SVPWM 零序注入
            q15_t phase_min = phase_a;
            q15_t phase_max = phase_a;

            if(phase_b < phase_min){phase_min = phase_b;}
            if(phase_c < phase_min){phase_min = phase_c;}

            if(phase_b > phase_max){phase_max = phase_b;}
            if(phase_c > phase_max){phase_max = phase_c;}

            const q15_t offset =
                Q15_HALF -
                ((phase_max + phase_min) >> 1);

            phase_a += offset;
            phase_b += offset;
            phase_c += offset;

            // Duty 限幅
            if(phase_a < 0){phase_a = 0;}
            else if(phase_a > Q15_ONE){phase_a = Q15_ONE;}

            if(phase_b < 0){phase_b = 0;}
            else if(phase_b > Q15_ONE){phase_b = Q15_ONE;}

            if(phase_c < 0){phase_c = 0;}
            else if(phase_c > Q15_ONE){phase_c = Q15_ONE;}

            // Q15 duty -> TIM2 CCR
            TIM2->CCR1 = (phase_a * 1600 + Q15_HALF) >> 15;
            TIM2->CCR2 = (phase_b * 1600 + Q15_HALF) >> 15;
            TIM2->CCR3 = (phase_c * 1600 + Q15_HALF) >> 15;
        }

        /**
         * @brief 自动检测编码器方向并校准零电角
         *
         * @return true 校准成功
         * @return false 校准失败
         */
        bool calibrate()
        {
            encoder_package encoder;

            // 等待编码器第一帧数据
            const uint32_t start = sys_time::get_ms_tick();
            while(sys_time::get_ms_tick() - start < 100)
            {
                as5600::update();
                if(as5600::get_package(encoder)){break;}
                sys_time::delay_ms(1);
            }

            if(!as5600::get_package(encoder)){return false;}

            // 检测编码器方向
            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,GPIO_PIN_SET
            );

            const int32_t start_count = encoder.full_count;

            for(uint16_t i = 0; i <= direction_steps; ++i)
            {
                const phase_t phase =
                    (phase_t)(
                        0xC000u +
                        (uint32_t)i *
                        65536u /
                        direction_steps
                    );

                svpwm(ALIGNMENT_UQ, phase);

                as5600::update();
                sys_time::delay_ms(2);
            }

            if(!as5600::get_package(encoder)){return false;}

            const int32_t delta = encoder.full_count - start_count;
            if(delta >= direction_min_count)
            {
                direction = 1;
            }
            else if(delta <= -direction_min_count)
            {
                direction = -1;
            }
            else
            {
                return false;
            }

            // 停止输出
            svpwm(0, 0);

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET
            );

            // 固定到 3π/2
            svpwm(ALIGNMENT_UQ, 0xC000);

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

            // 记录当前电角度作为零点
            zero_phase =
                (phase_t)(
                    (int32_t)direction *
                    pole_pairs *
                    (int32_t)get_phase(encoder.full_count)
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
         * @brief FOC 实时更新
         *
         * 由 TIM2 中断周期调用
         */
        void update()
        {
            encoder_package encoder;
            if(!as5600::get_package(encoder))
            {
                svpwm(0, 0);
                return;
            }

            const phase_t electrical_phase =
                (phase_t)(
                    (int32_t)direction *
                    pole_pairs *
                    (int32_t)get_phase(encoder.full_count) -
                    (int32_t)zero_phase
                );

            const int32_t torque_mNm = can_comm::get_target_mNm();

            const int32_t speed_mrad_s = (int32_t)direction * encoder.speed_mrad_s;

            // Torque -> normalized Uq
            const q15_t torque_uq = (torque_mNm * TORQUE_GAIN_Q8) >> 8;

            // Back-EMF compensation
            const q15_t bemf_uq = (speed_mrad_s * BEMF_GAIN_Q14) >> 14;
            
            svpwm(torque_uq + bemf_uq, electrical_phase);
        }
    }

    /**
     * @brief 初始化电机 FOC
     *
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool init()
    {
        // 保证启动阶段驱动关闭
        HAL_GPIO_WritePin(
            DRV_EN_GPIO_Port,
            DRV_EN_Pin,
            GPIO_PIN_RESET
        );

        // 清理中断状态
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC4);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);

        // 启动三相 PWM
        if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
           HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ||
           HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
        {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
            return false;
        }

        svpwm(0, 0);

        // 编码器方向和零电角自动校准
        if(!calibrate())
        {
            svpwm(0, 0);

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET
            );

            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

            return false;
        }

        // 校准完成，开启驱动
        HAL_GPIO_WritePin(
            DRV_EN_GPIO_Port,
            DRV_EN_Pin,
            GPIO_PIN_SET
        );

        // CH4 位于 PWM 中点，用于触发 FOC
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC4);

        if(HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
        {
            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET
            );

            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

            return false;
        }

        return true;
    }

}

/**
 * @brief TIM2 CH4 FOC 中断
 *
 * Center-aligned mode 1 下 CH4 每个 PWM 周期触发一次
 * 这里再二分，使 FOC 以 PWM 一半的频率运行
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
