#include "led.h"

#include "main.h"
#include "drivers/leds.h"
#include "devices/hw/motor.h"

namespace led
{
    namespace
    {
        leds normal_led(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        leds err_led(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_SET);

        bool error = false;

        /**
         * @brief 正常状态 LED 闪烁
         * 
         * 周期固定 1000ms
         *
         * @param tick_ms LED 点亮保持时间
         */
        void normal_led_blink(uint32_t tick_ms)
        {
            static uint8_t step = 0;
            static uint32_t step_tick = 0;

            step_tick += tick_ms;
            switch(step)
            {
                case 0:
                    normal_led.on();
                    if(step_tick >= tick_ms)
                    {
                        step_tick = 0;
                        step++;
                    }
                    break;

                case 1:
                    normal_led.off();
                    if(step_tick >= 1000 - tick_ms)
                    {
                        step_tick = 0;
                        step = 0;
                    }
                    break;
            }
        }
    }

    /**
     * @brief 设置错误指示状态
     *
     * @param state true 进入错误状态
     * @param state false 恢复正常状态
     */
    void set_error(bool state)
    {
        error = state;
    }

    /**
     * @brief 更新 LED 状态
     *
     * @param tick_ms 任务调用周期，单位 ms
     */
    void leds_proc(uint32_t tick_ms)
    {
        if(error || motor::has_fault())
        {
            normal_led.off();
            err_led.on();
            return;
        }

        normal_led_blink(tick_ms);
    }
}
