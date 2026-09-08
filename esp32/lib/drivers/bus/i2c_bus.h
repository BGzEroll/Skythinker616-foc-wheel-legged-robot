#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Arduino.h>
#include <Wire.h>

class i2c_bus {
    public:
        explicit i2c_bus(uint8_t bus_id = 0);

    public:
        void init();
        void read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
        void write_bytes(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len);
        TwoWire *get_TwoWire_handle() const;

    private:
        uint8_t bus_id;
};

#endif
