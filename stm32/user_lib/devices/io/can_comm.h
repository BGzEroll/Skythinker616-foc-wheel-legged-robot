#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "devices/hw/fixed_point.h"
#include "devices/hw/encoder.h"

namespace can_comm
{
    bool init();
    fixed::q16_t get_target_q16();
    bool send_feedback(const encoder_package &package);
}

#endif
