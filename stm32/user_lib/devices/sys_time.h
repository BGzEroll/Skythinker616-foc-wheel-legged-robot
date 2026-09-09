#ifndef SYS_TIME_H
#define SYS_TIME_H

#include <stdint.h>

namespace sys_time
{
    void delay_ms(uint32_t duration_ms);
    void delay_us(uint32_t duration_us);
    uint32_t get_ms_tick();
    uint32_t get_us_tick();
}

#endif
