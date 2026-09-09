#include "sys_time.h"

#include "stm32f1xx_hal.h"
#include "tim.h"

namespace sys_time
{

    namespace
    {
        bool initialized = false;

        /**
         * @brief 初始化定时器
         */
        void init()
        {
            if(initialized){return;}

            if(HAL_TIM_Base_Start(&htim1) != HAL_OK)
            {
                __disable_irq();
                while(true);
            }

            initialized = true;
        }
    }

    /**
     * @brief 延迟指定毫秒数
     *
     * @param duration_ms 延迟时长，单位毫秒
     */
    void delay_ms(uint32_t duration_ms)
    {
        HAL_Delay(duration_ms);
    }

    /**
     * @brief 延迟指定微秒数
     *
     * @param duration_us 延迟时长，单位微秒
     */
    void delay_us(uint32_t duration_us)
    {
        init();

        while(duration_us > 0)
        {
            const uint16_t duration =
                (duration_us > 30000) ? 30000 : (uint16_t)duration_us;

            const uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
            while((uint16_t)(__HAL_TIM_GET_COUNTER(&htim1) - start) < duration);
            duration_us -= duration;
        }
    }

    /**
     * @brief 获取系统毫秒时基计数
     *
     * @return 系统启动后的毫秒计数
     */
    uint32_t get_ms_tick()
    {
        return HAL_GetTick();
    }

    /**
     * @brief 获取定时器微秒时基计数
     * 
     * @return 定时器当前微秒计数值
     */
    uint32_t get_us_tick()
    {
        init();
        return __HAL_TIM_GET_COUNTER(&htim1);
    }

}
