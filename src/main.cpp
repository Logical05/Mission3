#include <Arduino.h>
#include <PanelSensor.h>
#include <Motor.h>
#include <PS2X_lib.h>

#define PS2_DAT 10
#define PS2_CMD 11
#define PS2_SEL 12
#define PS2_CLK 13

#define IR_LEFT 22
#define IR_MID_LEFT 23
#define IR_MID_RIGHT 24
#define IR_RIGHT 25

#define LED_BLUE 49
#define LED_RED 51
#define LED_GREEN 53

#define NUM_SENSORS 6
uint8_t PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};
PanelSensor panel(PINS, NUM_SENSORS);

Motor motor[2] = {
    Motor(7, 6, 5),     // Left Motor
    Motor(2, 3, 4),     // Right Motor
};

PS2X ps2x;
uint8_t error = 0;

void displayColor() {
    uint8_t value = panel.getColor();
    uint8_t color = value == 0 ? color : value;

    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    if      (color == 1) digitalWrite(LED_RED, HIGH);
    else if (color == 2) digitalWrite(LED_GREEN, HIGH);
    else                 digitalWrite(LED_BLUE, HIGH);
}

void setup() {
    Serial.begin(115200);
    panel.begin();
    motor[0].stop();
    motor[1].stop();
    
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    pinMode(IR_MID_LEFT, INPUT);
    pinMode(IR_MID_RIGHT, INPUT);

    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);

    while (true) {
        uint8_t left_state = digitalRead(IR_LEFT);
        uint8_t mid_left_state = digitalRead(IR_MID_LEFT);
        uint8_t mid_right_state = digitalRead(IR_MID_RIGHT);
        uint8_t right_state = digitalRead(IR_RIGHT);
        if (!left_state && !mid_left_state && !mid_right_state && !right_state) {
            motor[0].move(150);
            motor[1].move(150);
            continue;
        }
        if (!left_state && mid_left_state && !mid_right_state && !right_state) {
            motor[0].move(-100);
            motor[1].move(230);
            continue;
        }
        if (left_state && mid_left_state && !mid_right_state && !right_state) {
            motor[0].move(-255);
            motor[1].move(255);
            continue;
        }
        if (left_state && !mid_left_state && !mid_right_state && !right_state) {
            motor[0].move(-255);
            motor[1].move(255);
            continue;
        }
        if (!left_state && !mid_left_state && mid_right_state && !right_state) {
            motor[0].move(230);
            motor[1].move(-100);
            continue;
        }
        if (!left_state && !mid_left_state && mid_right_state && right_state) {
            motor[0].move(255);
            motor[1].move(-255);
            continue;
        }
        if (!left_state && !mid_left_state && !mid_right_state && right_state) {
            motor[0].move(255);
            motor[1].move(-255);
            continue;
        }
        if ((left_state && mid_left_state && mid_right_state && right_state) ||
            (left_state && mid_left_state && mid_right_state && !right_state) ||
            (!left_state && mid_left_state && mid_right_state && right_state)) {     
            motor[0].move(180);
            motor[1].move(180);

            delay(500);
            motor[0].stop();
            motor[1].stop();

            uint32_t start = millis();
            uint32_t time = 4 * 1e3;
            while (millis() - start < time) displayColor();
            break;
        }
    }
    Serial.println("Done");
}

void loop() {     
    displayColor();
    ps2x.read_gamepad(false, 0);
    if (ps2x.Button(PSB_PAD_UP)) {
        motor[0].move(255);
        motor[1].move(255); 
    } else if (ps2x.Button(PSB_PAD_DOWN)) {
        motor[0].move(-255);
        motor[1].move(-255);
    } else if (ps2x.Button(PSB_PAD_LEFT)) {
        motor[0].move(-255);
        motor[1].move(255);
    } else if (ps2x.Button(PSB_PAD_RIGHT)) {
        motor[0].move(255);
        motor[1].move(-255);
    } else {
        motor[0].stop();
        motor[1].stop();
    }

    delay(100);
}