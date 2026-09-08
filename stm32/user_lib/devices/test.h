#ifndef TEST_H
#define TEST_H

#include "stm32f1xx_hal.h"

void normal_led_blink(uint32_t tick, void *arg);
void encoder_test(uint32_t tick, void *arg);
void can_comm_test(uint32_t tick, void *arg);

void test_init();

#endif
