#include "test.h"

#include "drivers/leds.h"
#include "main.h"
#include "devices/hw/encoder.h"
#include "devices/io/can_comm.h"

static leds normal_led(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

void normal_led_blink(uint32_t tick, void *arg)
{
    static uint8_t step = 0;
    static uint32_t step_tick = 0;

    step_tick += tick;
    switch(step)
    {
        case 0:
            normal_led.on();
            if(step_tick >= tick)
            {
                step_tick = 0;
                step++;
            }
            break;

        case 1:
            normal_led.off();
            if(step_tick >= 1000 - tick)
            {
                step_tick = 0;
                step = 0;
            }
            break;
    }
}

void encoder_test(uint32_t tick, void *arg)
{
    static encoder_package package;

    if(as5600::update())
    {
        // 处理编码器数据包
    }
}

void can_comm_test(uint32_t tick, void *arg)
{
    can_comm_proc();
}

void test_init()
{
    as5600::init();
    can_comm_init();
}
