#ifndef UART_BUS_H
#define UART_BUS_H

#include <Arduino.h>

class uart_bus {
    public:
        explicit uart_bus(uint8_t bus_id = 0);

    public:
        void init();
        uint32_t read_bytes(uint8_t *buf, uint32_t max_len);
        void write_bytes(const uint8_t *buf, uint32_t len);
        HardwareSerial *get_HardwareSerial_handle() const;

    private:
        uint8_t bus_id;
};

#endif
