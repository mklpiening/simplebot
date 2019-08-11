#include <Arduino.h>

#include "Hardware.hpp"
#include "Motor.hpp"
#include "MotorSynchronizer.hpp"

Motor a0(PWMA_0, AIN1_0, AIN2_0, AENA_0, AENB_0, false);
Motor a1(PWMA_1, AIN1_1, AIN2_1, AENA_1, AENB_1, true);
Motor b0(PWMB_0, BIN1_0, BIN2_0, BENA_0, BENB_0, false);
Motor b1(PWMB_1, BIN1_1, BIN2_1, BENA_1, BENB_1, true);

Motor* motors[] = {&a0, &a1, &b0, &b1};
MotorSynchronizer sync(4, motors);

void setup() {
    Serial.begin(115200);
}

void loop() {
    a0.readPulsesFromSensor();
    a1.readPulsesFromSensor();
    b0.readPulsesFromSensor();
    b1.readPulsesFromSensor();

    if(sync.synchronize()) {
        Serial.write(0xFF);

        int64_t rotL = (b1.getPulses() + a1.getPulses()) / 2;
        Serial.write((uint8_t) (rotL & 0xFF));
        Serial.write((uint8_t) ((rotL >> 8) & 0xFF));
        Serial.write((uint8_t) ((rotL >> 16) & 0xFF));
        Serial.write((uint8_t) ((rotL >> 24) & 0xFF));
        Serial.write((uint8_t) ((rotL >> 32) & 0xFF));
        Serial.write((uint8_t) ((rotL >> 40) & 0xFF));
        Serial.write((uint8_t) ((rotL >> 48) & 0xFF));
        Serial.write((uint8_t) ((rotL >> 56) & 0xFF));

        int64_t rotR = (b0.getPulses() + a0.getPulses()) / 2;
        Serial.write((uint8_t) (rotR & 0xFF));
        Serial.write((uint8_t) ((rotR >> 8) & 0xFF));
        Serial.write((uint8_t) ((rotR >> 16) & 0xFF));
        Serial.write((uint8_t) ((rotR >> 24) & 0xFF));
        Serial.write((uint8_t) ((rotR >> 32) & 0xFF));
        Serial.write((uint8_t) ((rotR >> 40) & 0xFF));
        Serial.write((uint8_t) ((rotR >> 48) & 0xFF));
        Serial.write((uint8_t) ((rotR >> 56) & 0xFF));

        Serial.flush();
    }

    

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