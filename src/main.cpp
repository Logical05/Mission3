#include <Arduino.h>
#include <PanelSensor.h>
#include <Motor.h>
#include <PS2X_lib.h>

#define PS2_DAT 10
#define PS2_CMD 11
#define PS2_SEL 12
#define PS2_CLK 13

#define IR_LEFT 26
#define IR_MID_LEFT 27
#define IR_MID_RIGHT 28
#define IR_RIGHT 29

#define LED_BLUE 49
#define LED_RED 51
#define LED_GREEN 53

#define NUM_SENSORS 3
uint8_t PINS[NUM_SENSORS] = {A0, A2, A4};
PanelSensor panel(PINS, NUM_SENSORS);

Motor motor[2] = {
    Motor(7, 6, 5),     // Left Motor
    Motor(2, 3, 4),     // Right Motor
};

PS2X ps2x;
uint8_t ps2Error = 0;

bool isAuto = true;
uint8_t lastError = 0;

void displayIR()
{
    Serial.print(digitalRead(IR_LEFT));
    Serial.print("\t");
    Serial.print(digitalRead(IR_MID_LEFT));
    Serial.print("\t");
    Serial.print(digitalRead(IR_MID_RIGHT));
    Serial.print("\t");
    Serial.println(digitalRead(IR_RIGHT));
}

void displayColor() {
    uint8_t value = panel.getColor();
    uint8_t color = value == 0 ? color : value;

    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    // if      (color == 1) digitalWrite(LED_RED, HIGH);
    // else if (color == 2) digitalWrite(LED_GREEN, HIGH);
    // else                 digitalWrite(LED_BLUE, HIGH);
    if      (panel.getRaw(0) > panel.getRaw(1) && panel.getRaw(0) < panel.getRaw(2)) digitalWrite(LED_GREEN, HIGH);
    else if (panel.getRaw(0) > panel.getRaw(1) && panel.getRaw(1) < panel.getRaw(2)) digitalWrite(LED_BLUE, HIGH);
    else                                                                             digitalWrite(LED_RED, HIGH);
}

bool lineTrack() {
    int IR_Left_2_state = digitalRead(IR_LEFT);
    int IR_Left_1_state = digitalRead(IR_MID_LEFT);
    int IR_Right_1_state = digitalRead(IR_MID_RIGHT);
    int IR_Right_2_state = digitalRead(IR_RIGHT);

    if (IR_Left_2_state == 0 && IR_Left_1_state == 0 && IR_Right_1_state == 0 && IR_Right_2_state == 0)
    {
        motor[0].move(175);
        motor[1].move(175);
    }

    if (IR_Left_2_state == 1 && IR_Left_1_state == 1 && IR_Right_1_state == 1 && IR_Right_2_state == 0)
    {
        motor[0].move(-255);
        motor[1].move(255);
    }
    if (IR_Left_2_state == 0 && IR_Left_1_state == 1 && IR_Right_1_state == 1 && IR_Right_2_state == 1)
    {
        motor[0].move(255);
        motor[1].move(-255);
    }
    if (IR_Left_2_state == 0 && IR_Left_1_state == 1 && IR_Right_1_state == 0 && IR_Right_2_state == 0)
    {
        motor[0].move(-100);
        motor[1].move(230);
    }
    if (IR_Left_2_state == 1 && IR_Left_1_state == 1 && IR_Right_1_state == 0 && IR_Right_2_state == 0)
    {
        motor[0].move(-255);
        motor[1].move(255);
    }
    if (IR_Left_2_state == 1 && IR_Left_1_state == 0 && IR_Right_1_state == 0 && IR_Right_2_state == 0)
    {
        motor[0].move(-255);
        motor[1].move(255);
    }
    if (IR_Left_2_state == 0 && IR_Left_1_state == 0 && IR_Right_1_state == 1 && IR_Right_2_state == 0)
    {
        motor[1].move(-100);
        motor[0].move(230);
    }
    if (IR_Left_2_state == 0 && IR_Left_1_state == 0 && IR_Right_1_state == 1 && IR_Right_2_state == 1)
    {
        motor[1].move(-255);
        motor[0].move(255);
    }
    if (IR_Left_2_state == 0 && IR_Left_1_state == 0 && IR_Right_1_state == 0 && IR_Right_2_state == 1)
    {
        motor[1].move(-255);
        motor[0].move(255);
    }

    if (IR_Left_2_state == 1 && IR_Left_1_state == 1 && IR_Right_1_state == 1 && IR_Right_2_state == 1) {      
        motor[0].move(255);
        motor[1].move(255);
        delay(100);

        motor[0].stop();
        motor[1].stop();

        uint32_t start = millis();
        uint32_t time = 4 * 1e3;
        while (millis() - start < time) displayColor();
        return false;
    }

    // uint8_t error = (left_state + mid_left_state) - (mid_right_state + right_state);
    // uint8_t steering = (kp * error) + (kd * (error - lastError));
    // lastError = error;

    // int leftPower = constrain(speed - steering, -255, 255);
    // int rightPower = constrain(speed + steering, -255, 255);
    // motor[0].move(leftPower);
    // motor[1].move(rightPower); 
    return true;
}

void teleOp() {
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
    ps2Error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
}

void loop() {     
    if (isAuto) isAuto = lineTrack();
    else teleOp();
    // displayIR();
    // panel.rawSensor();
}