#include "led_dev.h"

led board_led(5);

/**
 * @brief 板载 led 灯闪烁函数
 */
static void board_led_blink(uint32_t tick)
{
    static uint8_t step = 0;
    static uint32_t step_tick = 0;

    step_tick += tick;
    switch(step)
    {
        case 0:
            board_led.on();
            if(step_tick >= tick)
            {
                step_tick = 0;
                step++;
            }
            break;
        
        case 1:
            board_led.off();
            if(step_tick >= 1000 - tick)
            {
                step_tick = 0;
                step = 0;
            }
            break;
    }
}

/**
 * @brief 板载 led 灯进程函数
 */
void board_led_dev_proc(uint32_t tick)
{
    board_led_blink(tick);
}
