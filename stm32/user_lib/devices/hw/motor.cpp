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

        // 电机 / 驱动固定参数
        constexpr uint8_t pole_pairs = 7;
        constexpr float bus_voltage = 12.0f;
        constexpr float voltage_limit = 4.0f;
        constexpr float pwm_period = 1600.0f;

        // SVPWM 最大可用 q 轴电压约为 Vbus / sqrt(3)
        constexpr float svpwm_limit = bus_voltage * 0.57735026918962576451f;
        constexpr float output_limit =
            voltage_limit < svpwm_limit
                ? voltage_limit
                : svpwm_limit;

        constexpr float bus_voltage_inv = 1.0f / bus_voltage;

        // 编码器校准参数
        constexpr float alignment_voltage = 3.0f;
        constexpr uint16_t direction_steps = 100;
        constexpr float direction_min_delta = two_pi / 100.0f;

        // CAN 指令超过该时间未更新则停止输出
        constexpr uint32_t command_timeout_ms = 100;

        int8_t direction = 0;
        float zero_angle = 0.0f;

        /**
         * @brief 将角度归一化到 [0, 2π)
         */
        float normalize_angle(float angle)
        {
            while(angle >= two_pi){angle -= two_pi;}
            while(angle < 0.0f){angle += two_pi;}
            return angle;
        }

        /**
         * @brief 正弦查表
         *
         * phase:
         * 0x0000 -> 0
         * 0x4000 -> π/2
         * 0x8000 -> π
         * 0xC000 -> 3π/2
         */
        float lookup_sin(uint16_t phase)
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

            return (float)(
                a + (((b - a) * fraction) >> 8)
            ) / 32768.0f;
        }

        /**
         * @brief 同时计算 sin / cos
         */
        void sin_cos(float angle, float &sine, float &cosine)
        {
            constexpr float rad_to_phase = 65536.0f / two_pi;

            const uint16_t phase = (uint16_t)(angle * rad_to_phase);

            sine = lookup_sin(phase);

            // cos(x) = sin(x + π/2)
            cosine = lookup_sin((uint16_t)(phase + 16384));
        }

        /**
         * @brief SpaceVectorPWM 核心路径
         *
         * voltage 为 q 轴目标电压
         * d 轴固定为 0
         *
         * 内部完成：
         * inverse Park
         * inverse Clarke
         * SVPWM 零序注入
         * TIM2 PWM 输出
         */
        void svpwm(float voltage, float angle)
        {
            // NaN 保护
            if(!(voltage == voltage))
            {
                voltage = 0.0f;
            }

            // 输出电压限制
            if(voltage > output_limit)
            {
                voltage = output_limit;
            }
            else if(voltage < -output_limit)
            {
                voltage = -output_limit;
            }

            angle = normalize_angle(angle);

            float sine, cosine;
            sin_cos(angle, sine, cosine);

            /*
             * Ud = 0
             *
             * inverse Park:
             *
             * alpha = -sin(theta) * Uq
             * beta  =  cos(theta) * Uq
             *
             * 提前归一化到母线电压。
             */
            const float uq = voltage * bus_voltage_inv;

            const float alpha = -sine * uq;
            const float beta = cosine * uq;

            // inverse Clarke
            float phase_a = alpha;
            float phase_b =
                -0.5f * alpha +
                sqrt_three_half * beta;
            float phase_c =
                -0.5f * alpha -
                sqrt_three_half * beta;

            // SVPWM 零序注入
            float phase_min = phase_a;
            float phase_max = phase_a;

            if(phase_b < phase_min){phase_min = phase_b;}
            if(phase_c < phase_min){phase_min = phase_c;}

            if(phase_b > phase_max){phase_max = phase_b;}
            if(phase_c > phase_max){phase_max = phase_c;}

            const float offset =
                0.5f -
                0.5f * (phase_max + phase_min);

            phase_a += offset;
            phase_b += offset;
            phase_c += offset;

            // 最终安全限幅
            if(phase_a < 0.0f){phase_a = 0.0f;}
            else if(phase_a > 1.0f){phase_a = 1.0f;}

            if(phase_b < 0.0f){phase_b = 0.0f;}
            else if(phase_b > 1.0f){phase_b = 1.0f;}

            if(phase_c < 0.0f){phase_c = 0.0f;}
            else if(phase_c > 1.0f){phase_c = 1.0f;}

            TIM2->CCR1 = (uint32_t)(phase_a * pwm_period + 0.5f);
            TIM2->CCR2 = (uint32_t)(phase_b * pwm_period + 0.5f);
            TIM2->CCR3 = (uint32_t)(phase_c * pwm_period + 0.5f);
        }

        /**
         * @brief 自动检测编码器方向并校准零电角
         */
        bool calibrate()
        {
            encoder_package encoder = {};

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

            const float start_angle = encoder.full_angle;

            for(uint16_t i = 0; i <= direction_steps; ++i)
            {
                svpwm(
                    alignment_voltage,
                    three_pi_2 +
                    two_pi *
                    (float)i /
                    (float)direction_steps
                );

                as5600::update();
                sys_time::delay_ms(2);
            }

            if(!as5600::get_package(encoder)){return false;}

            const float delta = encoder.full_angle - start_angle;
            if(delta > direction_min_delta)
            {
                direction = 1;
            }
            else if(delta < -direction_min_delta)
            {
                direction = -1;
            }
            else
            {
                return false;
            }

            // 电角度对齐
            svpwm(0.0f, 0.0f);

            HAL_GPIO_WritePin(
                DRV_EN_GPIO_Port,
                DRV_EN_Pin,
                GPIO_PIN_RESET
            );

            svpwm(
                alignment_voltage,
                three_pi_2
            );

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

            zero_angle = normalize_angle(
                (float)direction *
                (float)pole_pairs *
                encoder.angle
            );

            svpwm(0.0f, 0.0f);

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
            encoder_package encoder = {};
            if(!as5600::get_package(encoder))
            {
                svpwm(0.0f, 0.0f);
                return;
            }

            const float electrical_angle =
                (float)direction *
                (float)pole_pairs *
                encoder.angle -
                zero_angle;

            /*
             * 当前为 voltage torque mode
             *
             * 后续 estimated_current 只需要在这里：
             *
             * torque -> Iq -> Uq
             *
             * 最下面的 foc() 不需要改变
             */
            svpwm(can_comm::get_target(), electrical_angle);
        }
    }

    /**
     * @brief 初始化电机 FOC
     */
    bool init()
    {
        // 保证启动阶段驱动关闭
        HAL_GPIO_WritePin(
            DRV_EN_GPIO_Port,
            DRV_EN_Pin,
            GPIO_PIN_RESET
        );

        // 防止 TIM2 初始化阶段残留中断状态
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

        svpwm(0.0f, 0.0f);

        // 编码器方向和零电角自动校准
        if(!calibrate())
        {
            svpwm(0.0f, 0.0f);

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
