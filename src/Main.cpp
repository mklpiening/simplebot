#include <Arduino.h>

#include "Hardware.hpp"
#include "Motor.hpp"

Motor a0(PWMA_0, AIN1_0, AIN2_0, AENA_0, AENB_0, false);
Motor a1(PWMA_1, AIN1_1, AIN2_1, AENA_1, AENB_1, true);
Motor b0(PWMB_0, BIN1_0, BIN2_0, BENA_0, BENB_0, false);
Motor b1(PWMB_1, BIN1_1, BIN2_1, BENA_1, BENB_1, true);

void setup() {
  Serial.begin(115200);
}

void loop() {
  a0.readPulsesFromSensor();
  a1.readPulsesFromSensor();
  b0.readPulsesFromSensor();
  b1.readPulsesFromSensor();

  if (Serial.read() == 0xff) {
    delay(2);
    uint8_t dirs = Serial.read();
    if ((dirs & 0b11111100) == 0) {
      int16_t speedL = (((dirs & 0b00000010) >> 1) == 1 ? -1 : 1) * Serial.read();
      int16_t speedR = ((dirs  & 0b00000001) == 1 ? -1 : 1) * Serial.read();
      
      a0.setSpeed(speedR);
      b0.setSpeed(speedR);

      a1.setSpeed(speedL);
      b1.setSpeed(speedL);
    }
  }
}