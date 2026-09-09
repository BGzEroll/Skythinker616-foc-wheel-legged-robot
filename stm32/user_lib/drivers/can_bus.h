#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdint.h>

class can_bus {
    public:
        explicit can_bus(uint8_t bus_id = 0);

    public:
        void init();
        void register_receive_callback(void (*receive_cb)(uint32_t id, uint8_t *data));
        void send(uint32_t id, const uint8_t *data, uint8_t len);

    private:
        uint8_t bus_id;
};

#endif
