#include <Arduino.h>
#include <PanelSensor.h>

void PanelSensor::begin(uint16_t max[], uint16_t min[]) {
    for (uint8_t i = 0; i < SIZE; ++i) pinMode(PINS[i], INPUT);

    this->maxValue = max;
    this->minValue = min;
}

uint16_t PanelSensor::getRaw(uint8_t pin) {
    pin = constrain(pin, 0, SIZE);
    return analogRead(PINS[pin]);
}

uint16_t PanelSensor::getCalibrate(uint8_t pin) {
    pin = constrain(pin, 0, SIZE);
    return constrain(map(getRaw(pin), minValue[pin], maxValue[pin], 0, 1000), 0, 1000);
}

bool PanelSensor::inRange(int var, int ref, int plusMinus) {
    return (ref - plusMinus) <= var && var <= (ref + plusMinus);
}

bool PanelSensor::isLine(uint16_t range) {
    uint8_t num = SIZE - 1;
    return inRange(getRaw(0), getRaw(1), range) || inRange(getRaw(num - 1), getRaw(num), range);
}

byte PanelSensor::minRelated(uint8_t start) {
    uint16_t min = 1023;
    uint8_t index = 0;
    uint8_t end = start + (SIZE / 2);
    for (uint8_t i = start; i < end; ++i) {
        uint16_t value = getRaw(i);
        if (value >= min) continue;
        min = value;
        index = i;
    }
    
    byte colorCode = 0;
    for (uint8_t i = start; i < end; ++i) colorCode = colorCode << 1 | (index == i);
    return colorCode;
}

uint8_t PanelSensor::getColor() {
    byte colorCode = 0;
    if (isLine(2)) return 0;
    colorCode = minRelated() << (SIZE / 2) | minRelated(3);

    if      (colorCode == 0b001100) return 1;   // Red
    else if (colorCode == 0b010010) return 2;   // Green
    else if (colorCode == 0b100001) return 3;   // Blue
    else return 0;  // Online
}

uint16_t PanelSensor::getPosition(uint16_t track, uint16_t noise) {
    bool online = false;
    uint32_t avg = 0;
    uint16_t sum = 0;
    static uint16_t lastValue = 0;

    for (uint8_t i = 0; i < SIZE; ++i) {
        uint16_t values = getCalibrate(i);
        if (values >= track) online = true;
        if (values >= noise) {
            avg += (uint32_t)(values) * (i * 1000); 
            sum += values;
        }
    }
    
    if (online) {
        lastValue = avg / sum;
        return lastValue;
    }
    uint16_t max = (SIZE - 1) * 1000;
    return lastValue < max / 2 ? 0 : max;
}

void PanelSensor::calibrateSensor(uint16_t pauseTime, uint16_t samples) {
    Serial.println("CalibrateSensor");

    for (uint8_t i = 0; i < SIZE; ++i) {
        maxValue[i] = 0;
        minValue[i] = 1023;
    }
    for (uint16_t startSamp = 0; startSamp <= samples; ++startSamp) {
        for (uint8_t i = 0; i < SIZE; ++i) {
            uint16_t value = getRaw(i);
            maxValue[i] = max(value, maxValue[i]);
            minValue[i] = min(value, minValue[i]);
        }
        delay(pauseTime);
        Serial.print(".");
    }
    for (uint8_t i = 0; i < SIZE; ++i) {
        maxValue[i] -= MAX_SAFETY;
        minValue[i] += MIN_SAFETY;
    }
    Serial.println("FINISH_CalibrateSensor");

    Serial.print("Max = { ");
    for (uint8_t i = 0; i < SIZE; ++i) {
        Serial.print(maxValue[i]);
        Serial.print(", ");
    }
    Serial.println("};");

    Serial.print("Min = { ");
    for (uint8_t i = 0; i < SIZE; ++i) {
        Serial.print(minValue[i]);
        Serial.print(", ");
    }
    Serial.println("};");
}

void PanelSensor::rawSensor() {
    for (uint8_t i = 0; i < SIZE; ++i) {
        Serial.print(getRaw(i));
        Serial.print("\t");
    }
    Serial.println();
}