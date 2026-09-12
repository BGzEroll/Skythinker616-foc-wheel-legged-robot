// #include "test.h"

// #include "drivers/leds.h"
// #include "main.h"
// #include "devices/hw/encoder.h"
// #include "devices/io/can_comm.h"

// static leds normal_led(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

// void normal_led_blink(uint32_t tick)
// {
//     static uint8_t step = 0;
//     static uint32_t step_tick = 0;

//     step_tick += tick;
//     switch(step)
//     {
//         case 0:
//             normal_led.on();
//             if(step_tick >= tick)
//             {
//                 step_tick = 0;
//                 step++;
//             }
//             break;

//         case 1:
//             normal_led.off();
//             if(step_tick >= 1000 - tick)
//             {
//                 step_tick = 0;
//                 step = 0;
//             }
//             break;
//     }
// }

// void can_comm_test()
// {
//     static encoder_package package;
//     as5600::get_package(package);
//     can_comm::send_feedback(package);
// }

// void test_init()
// {
//     // can_comm_init();
// }
