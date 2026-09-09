#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <Arduino.h>
#include "driver/twai.h"

class can_bus {
    public:
        can_bus(uint8_t bus_id = 0);

    public:
        void init();
        void receive(void (*receive_cb)(uint32_t id, const uint8_t *data, uint8_t len));
        void send(uint32_t id, const uint8_t *data, uint8_t len);

    private:
        uint8_t bus_id;
};

#endif
