#ifndef LED_DEV_H
#define LED_DEV_H

#include "led.h"

extern led board_led;

void board_led_dev_proc(uint32_t tick);

#endif
