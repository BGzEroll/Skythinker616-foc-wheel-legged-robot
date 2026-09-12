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
        using q15_t = int32_t;      // Q15 标度：1.0 = 32768，使用 int32_t 保存
        using phase_t = uint16_t;

        constexpr q15_t Q15_ONE = 32768;
        constexpr q15_t Q15_HALF = 16384;

        // 电机参数
        constexpr int32_t pole_pairs = 7;

        // Q15 常量
        constexpr q15_t SQRT3_HALF = 28378;     // sqrt(3) / 2
        constexpr q15_t OUTPUT_LIMIT = 18919;   // SVPWM 线性区 Uq/Vbus = 1/sqrt(3)
        constexpr q15_t ALIGNMENT_UQ = 8192;    // 校准电压 3V / 12V

        // 编码器校准参数
        constexpr uint16_t direction_steps = 100;
        constexpr int32_t direction_min_count = 41;
        constexpr uint32_t alignment_settle_ms = 300;
        constexpr uint16_t zero_sample_count = 32;

        // 角度预测
        constexpr uint16_t max_predict_us = 500;

        // speed_mrad_s * us -> 16-bit mechanical phase
        // 65536 / (2π * 1e9) ≈ 175 / 2^24
        constexpr int32_t PHASE_PRED_GAIN_Q24 = 175;

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
         * @return Q15 标度正弦值，1.0 = 32768
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
         * @brief 保持当前电角度并持续更新编码器
         *
         * @param time_ms 保持时间，单位 ms
         */
        void wait_encoder(uint32_t time_ms)
        {
            const uint32_t start = sys_time::get_ms_tick();
            while(sys_time::get_ms_tick() - start < time_ms)
            {
                as5600::update();
                sys_time::delay_ms(1);
            }
        }

        /**
         * @brief 获取多个新编码器样本的平均累计计数
         *
         * @param average_count 输出平均累计计数
         * @param sample_count 采样数量
         *
         * @return true 采样成功
         * @return false 采样超时
         */
        bool get_average_count(int32_t &average_count, uint16_t sample_count)
        {
            int64_t sum = 0;
            uint16_t collected = 0;

            const uint32_t start = sys_time::get_ms_tick();
            while(collected < sample_count)
            {
                // 只统计真正的新 DMA 样本
                if(as5600::update())
                {
                    encoder_package encoder;
                    if(as5600::get_package(encoder))
                    {
                        sum += encoder.full_count;
                        collected++;
                    }
                }

                // 防止 I2C 异常时永久卡死初始化
                if(sys_time::get_ms_tick() - start > 100)
                {
                    return false;
                }
            }

            average_count = (int32_t)(sum / sample_count);

            return true;
        }

        /**
         * @brief 根据编码器样本年龄预测当前机械角
         *
         * @param encoder 最新编码器数据
         *
         * @return 预测后的 16-bit 机械角
         */
        inline phase_t get_predicted_phase(const encoder_package &encoder)
        {
            const uint16_t now_us = (uint16_t)sys_time::get_us_tick();
            uint16_t age_us = (uint16_t)(now_us - encoder.timestamp_us);

            if(age_us > max_predict_us)
            {
                age_us = max_predict_us;
            }

            const int32_t phase_advance =
                (int32_t)(
                    (
                        (int64_t)encoder.speed_mrad_s *
                        age_us *
                        PHASE_PRED_GAIN_Q24
                    ) >> 24);

            return (phase_t)(
                    (int32_t)get_phase(encoder.full_count) +
                    phase_advance);
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

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_SET);

            // 先固定到扫描起点
            svpwm(ALIGNMENT_UQ, 0xC000);
            wait_encoder(alignment_settle_ms);
            if(!as5600::get_package(encoder)){return false;}

            const int32_t start_count = encoder.full_count;

            // 正向扫描一个完整电角周期
            for(uint16_t i = 0; i <= direction_steps; ++i)
            {
                const phase_t phase =
                    (phase_t)(
                        0xC000u +
                        (uint32_t)i *
                        65536u /
                        direction_steps);

                svpwm(ALIGNMENT_UQ, phase);

                as5600::update();
                sys_time::delay_ms(2);
            }

            if(!as5600::get_package(encoder)){return false;}

            const int32_t forward_count = encoder.full_count;

            // 沿相同路径反向扫描
            for(int32_t i = direction_steps; i >= 0; --i)
            {
                const phase_t phase =
                    (phase_t)(
                        0xC000u +
                        (uint32_t)i *
                        65536u /
                        direction_steps);

                svpwm(ALIGNMENT_UQ, phase);

                as5600::update();
                sys_time::delay_ms(2);
            }

            if(!as5600::get_package(encoder)){return false;}

            const int32_t end_count = encoder.full_count;
            const int32_t forward_delta = forward_count - start_count;
            const int32_t reverse_delta = end_count - forward_count;

            // 正扫与反扫必须表现出相反的运动方向
            if(forward_delta >= direction_min_count &&
                reverse_delta <= -direction_min_count)
            {
                direction = 1;
            }
            else if(forward_delta <= -direction_min_count &&
                reverse_delta >= direction_min_count)
            {
                direction = -1;
            }
            else
            {
                return false;
            }

            // 反向扫描结束后已经回到 0xC000，再保持一段时间，让转子完全稳定
            svpwm(ALIGNMENT_UQ, 0xC000);
            wait_encoder(alignment_settle_ms);

            // 对多个真实的新编码器样本取平均
            int32_t average_count;
            if(!get_average_count(average_count, zero_sample_count))
            {
                return false;
            }

            zero_phase =
                (phase_t)(
                    (int32_t)direction *
                    pole_pairs *
                    (int32_t)get_phase(average_count));

            svpwm(0, 0);

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET);

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

            const phase_t mechanical_phase = get_predicted_phase(encoder);

            const phase_t electrical_phase =
                (phase_t)(
                    (int32_t)direction *
                    pole_pairs *
                    (int32_t)mechanical_phase -
                    (int32_t)zero_phase);

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
            GPIO_PIN_RESET);

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
                GPIO_PIN_RESET);

            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

            return false;
        }

        // 校准完成，开启驱动
        HAL_GPIO_WritePin(
            DRV_EN_GPIO_Port,
            DRV_EN_Pin,
            GPIO_PIN_SET);

        // CH4 位于 PWM 中点，用于触发 FOC
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC4);

        if(HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK)
        {
            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET);

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
