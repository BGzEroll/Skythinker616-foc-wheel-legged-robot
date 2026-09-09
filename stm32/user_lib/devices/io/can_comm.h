#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "devices/hw/encoder.h"

struct target_package
{
    uint32_t timestamp_ms = 0;
    uint32_t sequence = 0;

    float voltage = 0.0f;
};

namespace can_comm
{
    bool init();
    bool get_package(target_package &snapshot);
    bool send_feedback(const encoder_package &package);
}

#endif
