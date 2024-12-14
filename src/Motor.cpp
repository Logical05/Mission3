#include "Motor.h"

Motor::Motor(uint8_t EN, uint8_t IN1, uint8_t IN2): EN(EN), IN1(IN1), IN2(IN2) {
    pinMode(EN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    stop();
}

void Motor::move(int speed) {
    bool dir = speed >= 0;

    speed = abs(speed);
    speed = constrain(speed, 0, 255);  
    if (speed == 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, dir);
        digitalWrite(IN2, !dir);
    }
    analogWrite(EN, speed);
}

void Motor::stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN, 0);
}