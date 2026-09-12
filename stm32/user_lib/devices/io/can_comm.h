#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "devices/hw/encoder.h"

namespace can_comm
{
    bool init();
    int32_t get_target_mNm();
    bool send_feedback(const encoder_package &package);
}

#endif
