#ifndef LED_H
#define LED_H

#include <Arduino.h>

class led {
    public:
        led(uint8_t pin);

    public:
        void init();

    public:
        void on();
        void off();
        void toggle();

    private:
        uint8_t pin;
};

#endif
