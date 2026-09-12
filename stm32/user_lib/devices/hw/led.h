#ifndef LED_H
#define LED_H

#include <stdint.h>

namespace led
{
    void set_error(bool state);
    void leds_proc(uint32_t tick_ms);
}

#endif
