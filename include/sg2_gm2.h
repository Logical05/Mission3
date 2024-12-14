#include <POP32.h>
#include <PathRecord.h>
#include <encoder.h>
#include "EEPROMValue.h"
#include <Gyro.h>

#define NUM_SENSORS 8

int LastError;
int MinValue[NUM_SENSORS];
int MaxValue[NUM_SENSORS];
uint8_t F_PIN[NUM_SENSORS] = {0, 1, 2, 3, 4, 5, 6, 7};
int F[NUM_SENSORS];
int LTurnSpdL, LTurnSpdR, TurnDelayL;
int RTurnSpdL, RTurnSpdR, TurnDelayR;
int LineColor = 0;

unsigned long timer;
float timerst;
float timersp;


int degerr;
int countS = 0;
int countR = 0;
bool curvestate = 0;
bool rampstate = 0;

int setSpoint = 90;
int Sinput = 0;
int Soutput = 0;
int SlastInput = 0;
int Sintegral = 0;

volatile long encoderPos = 0;
int SPower = setSpoint;
int Error = 90;
const int encoderPinA = PB3;

void updateEncoder() {
  encoderPos++;
}

void Beep(int delayb) {
  sound(1000, delayb);
}

void ReadSensor() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = analogRead(F_PIN[i]);
  }
}

void ReadCalibrate() {
  ReadSensor();
  for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = constrain(map(F[i], MinValue[i], MaxValue[i], 0, 1000), 0, 1000);
  }
}

void CalibrateSensor(int pauseTime, int samples) {
  fd2(30, -30);
  for (int i = 0; i < NUM_SENSORS; i++) {
    MinValue[i] = 1023;
    MaxValue[i] = 0;
  }
  for (int startSamp = 0; startSamp <= samples; startSamp++) {
    ReadSensor();
    for (int i = 0; i < NUM_SENSORS; i++) {
      MinValue[i] = min(F[i], MinValue[i]);
      MaxValue[i] = max(F[i], MaxValue[i]);
    }
    delay(pauseTime);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    MinValue[i] += 20;
    MaxValue[i] -= 80;
  }
  ao();
}

void SensorValue() {
  // MinValue[0] = minF0;
  // MinValue[1] = minF1;
  // MinValue[2] = minF2;
  // MinValue[3] = minF3;
  // MinValue[4] = minF4;
  // MinValue[5] = minF5;
  // MinValue[6] = minF6;
  // MinValue[7] = minF7;

  // MaxValue[0] = maxF0;
  // MaxValue[1] = maxF1;
  // MaxValue[2] = maxF2;
  // MaxValue[3] = maxF3;
  // MaxValue[4] = maxF4;
  // MaxValue[5] = maxF5;
  // MaxValue[6] = maxF6;
  // MaxValue[7] = maxF7;
  loadCalibrationFromEEPROM(MinValue, MaxValue, NUM_SENSORS, 900);
}

void RobotSetup() {
  Serial.begin(115200);
  Wire.begin();
  servo(1, 90);
  for (int freq = 1500; freq <= 4000; freq += 500) {
    sound(freq, 40);
  }
  secondTest();
  setupGyro();
  oled.clear();
  oled.textSize(1);
  pinMode(encoderPinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, RISING);
  SensorValue();
}

void OK() {
  waitSW_OK();
  Beep(100);
}

int OK_PUSH() {
  return SW_OK();
}

void CalibrateRobotSensor() {
  Beep(100);
  delay(200);
  CalibrateSensor(20, 200);
  Beep(100);
  saveCalibrationToEEPROM(MinValue, MaxValue, NUM_SENSORS, 900);
  // while (1);
}

void SerialSensor() {
  while (1) {
    ReadSensor();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(50);
    if (OK_PUSH() == 1) {
      Beep(40);
      break;
    }
  }
}

void SerialCalibrate() {
  while (1) {
    ReadCalibrate();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }
    Serial.println("");
    delay(100);
    if (OK_PUSH() == 1) {
      Beep(40);
      break;
    }
  }
}

int readPosition(int Track, int noise) {
  unsigned char online = 0;
  unsigned long avg = 0;
  unsigned int sum = 0;
  static int last_value = 0;
  ReadCalibrate();
  for (int i = 0; i < NUM_SENSORS; i++) {
    int values = F[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    return last_value < (NUM_SENSORS - 1) * 1000 / 2 ? 0 : (NUM_SENSORS - 1) * 1000;
  }
  last_value = avg / sum;
  return last_value;
}

int SensorControl(float Skp, float Skd) {
  int Pos = readPosition(300, 50);
  Sinput = map(Pos, 0, 7000, 0, 180);
  int Serror = Error - Sinput;
  Soutput = (Skp * Serror) + (Skd * (Sinput - SlastInput));
  SlastInput = Sinput;
  Error = setSpoint - Soutput;
  servo(1, Error);
  return Error;
}

int sp;
int rp;

void PID(int Speed, float Mkp, float Mkd, float Skp, float Skd, bool status) {
  degerr = SensorControl(Skp, Skd);
  int Error = setSpoint - degerr;
  int PID_Value = (Mkp * Error) + (Mkd * (Error - LastError));
  LastError = Error;

  int LeftPower = constrain(Speed - PID_Value, -100, 100);
  int RightPower = constrain(Speed + PID_Value, -100, 100);

  fd2(LeftPower, RightPower);

  if (!status) {
    mpu.update();

    if (abs(degerr - 90) >= JD) { // 10
      if (!curvestate) {
        sp = encoderPos;
        curvestate = 1;
      }
    } else if (curvestate) {
      recordPath(sp, encoderPos);
      curvestate = 0;
    }

    if (abs(mpu.getAngleX()) >= JG) { //13
      if (!rampstate) {
        rp = encoderPos;
        rampstate = 1;
      }
    } else if (abs(mpu.getAngleX()) <= 3 && rampstate) {
      recordPath1(rp, encoderPos);
      rampstate = 0;
    }
  }
}

void TurnLeft() {
  fd2(-LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
  while (1) {
    fd2(-LTurnSpdL, LTurnSpdR);
    ReadCalibrate();
    if (F[2] >= 500) {
      ao();
      break;
    }
  }
}

void TurnRight() {
  fd2(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    fd2(RTurnSpdL, -RTurnSpdR);
    ReadCalibrate();
    if (F[5] >= 500) {
      ao();
      break;
    }
  }
}

void TrackSelect(int spd, char x) {
  ReadCalibrate();
  switch (x) {
    case 's':
      ao();
      motor(3, 0);
      break;
    case 'p':
      fd2(spd, spd);
      delay(30);
      while (1) {
        fd2(spd, spd);
        ReadCalibrate();
        if (F[0] < 550 && F[7] < 550) {
          fd2(spd, spd);
          delay(5);
          break;
        }
      }
      break;
    case 'l':
      TurnLeft();
      break;
    case 'r':
      TurnRight();
      break;
  }
}

void TrackCross(int Speed, float Mkp, float Mkd, char select, float Skp, float Skd, bool status) {
  countS = 0;
  countR = 0;
  rampstate = 0;
  curvestate = 0;

  while (1) {
    PID(Speed, Mkp, Mkd, Skp, Skd, status);
    ReadCalibrate();
    mpu.update();

    if (status) {
      if (encoderPos >= path1[countR].st2 - ennoiseR && encoderPos <= path1[countR].sp2) {
        if (mpu.getAngleX() < 0) {
          Speed = Vmin + SpeedUP ;
        } else {
          Speed = Vmin + SpeedDOWN ;
        }
      } else {
        Speed = Vmax;
        if (encoderPos >= path1[countR].sp2) countR++;
        if (encoderPos >= path[countS].encoderst - enTurn && encoderPos <= path[countS].encodersp) {
          Speed = Vmin;
        } else {
          Speed = Vmax;
          if (encoderPos >= path[countS].encodersp) countS++;
          if (encoderPos >= path1[countR].sp2) countR++;
        }
      }
    }

    if (encoderPos > (Smax * 255) - Snoise) {
      recordPath1(encoderPos, encoderPos);
      break;
    }
    if (F[0] > 550 || F[1] > 550 || F[2] > 550 || F[3] > 550 || F[4] > 550 || F[5] > 550 || F[6] > 550 || F[7] > 550) {
      timer = millis();

    }
    if (F[0] < 550 && F[1] < 550 && F[2] < 550 && F[3] < 550 && F[4] < 550 && F[5] < 550 && F[6] < 550 && F[7] < 550) {
      if (millis() - timer >= 300) {
        recordPath1(encoderPos, encoderPos);
        break;
      }
    }
    if (F[0] > 700 && F[1] > 700 && F[2] > 700 && F[3] > 700 && F[4] > 700 && F[5] > 700 && F[6] > 700 && F[7] > 700) {
      recordPath1(encoderPos, encoderPos);
      break;
    }
  }
  TrackSelect(Speed, select);
}

