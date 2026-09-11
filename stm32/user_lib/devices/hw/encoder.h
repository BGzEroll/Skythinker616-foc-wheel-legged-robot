#ifndef ENCODER_H
#define ENCODER_H

#include "fixed_point.h"
#include <stdint.h>

struct encoder_package
{
    uint16_t timestamp_us = 0;       // 低 16 位，保留 CAN 反馈协议
    uint32_t timestamp_ms = 0;       // 单调毫秒时基，用于新鲜度判断
    uint16_t angle_phase = 0;        // 一圈相位：0x0000...0xFFFF

    int32_t full_count = 0;
    fixed::q16_t speed_q16 = 0;      // rad/s，Q16.16
    uint32_t sequence = 0;
};

namespace as5600
{
    bool init();
    bool update();
    bool get_package(encoder_package &snapshot);
}

#endif
