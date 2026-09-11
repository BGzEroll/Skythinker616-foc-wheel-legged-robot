#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "devices/hw/encoder.h"

namespace can_comm
{
    bool init();
    float get_target();
    bool send_feedback(const encoder_package &package);
}

#endif
