#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

struct encoder_package
{
    uint16_t timestamp_us = 0;

    int32_t full_count = 0;
    float angle = 0.0f;
    float full_angle = 0.0f;
};

namespace as5600
{
    bool init();
    bool update();
    bool get_package(encoder_package &snapshot);
}

#endif
