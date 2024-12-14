#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

class Motor {
    private:
        const uint8_t EN;
        const uint8_t IN1;
        const uint8_t IN2;

    public:
        Motor(uint8_t EN, uint8_t IN1, uint8_t IN2);
        void move(int speed);
        void stop();
};

#endif