#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

struct encoder_package
{
    uint32_t timestamp_ms = 0;
    uint32_t sequence = 0;

    float angle = 0.0f;
    float full_angle = 0.0f;
    float speed = 0.0f;
};

namespace as5600
{
    bool init();
    bool update();
    bool get_package(encoder_package &snapshot);
}

#endif
