#ifndef TEST_H
#define TEST_H

#include <stdint.h>

void normal_led_blink(uint32_t tick);
void encoder_test(uint32_t tick, void *arg);
void can_comm_test();

void test_init();

#endif
