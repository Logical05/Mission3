#ifndef PanelSensor_h
#define PanelSensor_h

#include <Arduino.h>

#define THRESHOLD_MULTIPLIER 1.1
#define MAX_SAFETY 80
#define MIN_SAFETY 20

class PanelSensor {
    private:
        const uint8_t* PINS;
        const uint8_t SIZE;
        uint16_t* maxValue;
        uint16_t* minValue;

        bool inRange(int var, int ref, int plusMinus);
        bool isLine(uint16_t range);
        byte minRelated(uint8_t start = 0);
        
    public:
        PanelSensor(uint8_t pins[], const uint8_t size): PINS(pins), SIZE(size) {}
        void begin(uint16_t max[] = NULL, uint16_t min[] = NULL);
        uint16_t getRaw(uint8_t pin);
        uint16_t getCalibrate(uint8_t pin);
        uint8_t getColor();
        uint16_t getPosition(uint16_t track, uint16_t noise);
        void calibrateSensor(uint16_t pauseTime, uint16_t samples);
        void rawSensor();
};

#endif