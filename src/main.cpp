#include <Arduino.h>

#define PWMA_0 3
#define AIN1_0 2
#define AIN2_0 4
#define AENA_0 A4
#define AENB_0 A5

#define PWMB_0 5
#define BIN1_0 7
#define BIN2_0 6
#define BENA_0 A1
#define BENB_0 A0

#define PWMA_1 11
#define AIN1_1 13
#define AIN2_1 12
#define AENA_1 A6
#define AENB_1 A7

#define PWMB_1 9
#define BIN1_1 10
#define BIN2_1 8
#define BENA_1 A3
#define BENB_1 A2


#define M_A0 0
#define M_B0 1
#define M_A1 2
#define M_B1 3


#define MUL_A0 1.0
#define MUL_B0 1.0
#define MUL_A1 0.9
#define MUL_B1 1.0


void setSpeed(uint8_t motor, int32_t speed) {
  char pin1 = 13;
  char pin2 = 13;
  char pinPwm = 13;

  switch (motor)
  {
  case M_A0:
    pin1 = AIN1_0;
    pin2 = AIN2_0;
    pinPwm = PWMA_0;

    speed *= MUL_A0;
    break;

  case M_B0:
    pin1 = BIN1_0;
    pin2 = BIN2_0;
    pinPwm = PWMB_0;
    
    speed *= MUL_B0;
    break;

  case M_A1:
    pin1 = AIN1_1;
    pin2 = AIN2_1;
    pinPwm = PWMA_1;
    
    speed *= MUL_A1;
    break;

  case M_B1:
    pin1 = BIN1_1;
    pin2 = BIN2_1;
    pinPwm = PWMB_1;
    
    speed *= MUL_B1;
    break;
  
  default:
    return;
  }

  if (speed == 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, HIGH);
    analogWrite(pinPwm, 0);
  } else if (speed < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pinPwm, -speed);
  } else {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pinPwm, speed);
  }
}

void drive(int32_t speedL, int32_t speedR) {
  setSpeed(M_A1, -speedL);
  setSpeed(M_B1, -speedL);

  setSpeed(M_A0, speedR);
  setSpeed(M_B0, speedR);
}

void calcRotation(uint8_t ena, uint8_t enb, bool& previousState, int32_t& rotation, bool invert) {
  if (analogRead(ena) > 512) {
    if (previousState == true) {
      if (analogRead(enb) < 512) {
        if (invert) {
          rotation--;
        } else {
          rotation++;
        }
      } else {
        if (invert) {
          rotation++;
        } else {
          rotation--;
        }
      }
      Serial.println(rotation);

      previousState = false;
    }
  } else {
    if (previousState == false) {
      previousState = true;
    }
  }
}

void setup() {
  pinMode(PWMA_0, OUTPUT);
  pinMode(AIN1_0, OUTPUT);
  pinMode(AIN2_0, OUTPUT);
  pinMode(AENA_0, INPUT);
  pinMode(AENB_0, INPUT);

  pinMode(PWMB_0, OUTPUT);
  pinMode(BIN1_0, OUTPUT);
  pinMode(BIN2_0, OUTPUT);
  pinMode(BENA_0, INPUT);
  pinMode(BENB_0, INPUT);

  pinMode(PWMA_1, OUTPUT);
  pinMode(AIN1_1, OUTPUT);
  pinMode(AIN2_1, OUTPUT);
  pinMode(AENA_1, INPUT);
  pinMode(AENB_1, INPUT);

  pinMode(PWMB_1, OUTPUT);
  pinMode(BIN1_1, OUTPUT);
  pinMode(BIN2_1, OUTPUT);
  pinMode(BENA_1, INPUT);
  pinMode(BENB_1, INPUT);

  drive(100, 100);

  Serial.begin(115200);
}

int32_t rotA0 = 0;
bool lastA0 = false;

int32_t rotA1 = 0;
bool lastA1 = false;

int32_t rotB0 = 0;
bool lastB0 = false;

int32_t rotB1 = 0;
bool lastB1 = false;

void loop() {
  calcRotation(AENA_0, AENB_0, lastA0, rotA0, true);
  calcRotation(AENA_1, AENB_1, lastA1, rotA1, true);
  calcRotation(BENA_0, BENB_0, lastB0, rotB0, true);
  calcRotation(BENA_1, BENB_1, lastB1, rotB1, false);
  
  if (Serial.read() == 0xff) {
    delay(2);
    uint8_t dirs = Serial.read();
    if ((dirs & 0b11111100) == 0) {
      int speedL = (((dirs & 0b00000010) >> 1) == 1 ? -1 : 1) * Serial.read();
      int speedR = ((dirs  & 0b00000001) == 1 ? -1 : 1) * Serial.read();
      
      drive(speedL, speedR);
    }
  }
}