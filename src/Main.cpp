#include "Hardware.hpp"
#include "Motor.hpp"
#include "MotorPID.hpp"

#include <Arduino.h>

// #define CALIBRATION

// right side
MotorPID g_a0(PWMA_0, AIN1_0, AIN2_0, AENA_0, AENB_0, false, 100, KP_A0, KI_A0, KD_A0);
MotorPID g_b0(PWMB_0, BIN1_0, BIN2_0, BENA_0, BENB_0, false, 100, KP_B0, KI_B0, KD_B0);

// left side
MotorPID g_a1(PWMA_1, AIN1_1, AIN2_1, AENA_1, AENB_1, true, 100, KP_A1, KI_A1, KD_A1);
MotorPID g_b1(PWMB_1, BIN1_1, BIN2_1, BENA_1, BENB_1, true, 100, KP_B1, KI_B1, KD_B1);

unsigned long g_lastOdomTransTime = 0;

float g_lastRotL = 0;
float g_lastRotR = 0;

bool checkParity(int64_t data) {
    int numOnes = 0;
    for (int i = 0; i < 64; i++) {
        if (data & (1 << i)) {
            numOnes++;
        }  
    }

    return numOnes % 2 == 0;
}

void setup()
{
    g_a0.setRPS(0);
    g_b0.setRPS(0);
    g_a1.setRPS(0);
    g_b1.setRPS(0);

    Serial.begin(115200);
}

#ifdef CALIBRATION
float speed = 0;
int iteration = 0;
#endif

void loop()
{
    g_a0.handle();
    g_b0.handle();
    g_a1.handle();
    g_b1.handle();

    if (millis() - g_lastOdomTransTime > 120)
    {
        g_lastOdomTransTime = millis();

#ifndef CALIBRATION
        Serial.write(0xFF);

        float rotL    = (float)(g_b1.getPulses() + g_a1.getPulses()) / (PULSES_PER_ROTATION * 2);
        int64_t dRotL = (rotL - g_lastRotL) * 10000;
        g_lastRotL    = rotL;
        Serial.write((uint8_t)(dRotL & 0xFF));
        Serial.write((uint8_t)((dRotL >> 8) & 0xFF));
        Serial.write((uint8_t)((dRotL >> 16) & 0xFF));
        Serial.write((uint8_t)((dRotL >> 24) & 0xFF));
        Serial.write((uint8_t)((dRotL >> 32) & 0xFF));
        Serial.write((uint8_t)((dRotL >> 40) & 0xFF));
        Serial.write((uint8_t)((dRotL >> 48) & 0xFF));
        Serial.write((uint8_t)((dRotL >> 56) & 0xFF));

        float rotR    = (float)(g_b0.getPulses() + g_a0.getPulses()) / (PULSES_PER_ROTATION * 2);
        int64_t dRotR = (rotR - g_lastRotR) * 10000;
        g_lastRotR    = rotR;
        Serial.write((uint8_t)(dRotR & 0xFF));
        Serial.write((uint8_t)((dRotR >> 8) & 0xFF));
        Serial.write((uint8_t)((dRotR >> 16) & 0xFF));
        Serial.write((uint8_t)((dRotR >> 24) & 0xFF));
        Serial.write((uint8_t)((dRotR >> 32) & 0xFF));
        Serial.write((uint8_t)((dRotR >> 40) & 0xFF));
        Serial.write((uint8_t)((dRotR >> 48) & 0xFF));
        Serial.write((uint8_t)((dRotR >> 56) & 0xFF));

        Serial.flush();
#else
        
        Serial.println((int) (g_a0.getCurrentRPS() * 1000));
        Serial.println((int) (g_a0.getPWM() * 1000.0 / 255));
        Serial.println((int) (speed * 1000));

        if (iteration == 20) {
            speed = -0.5; 
        }

        if (iteration == 60) {
            speed = 0.5; 
        }

        if (iteration == 100) {
            speed = 2; 
        }

        if (iteration > 140 && iteration < 180) {
            speed -= 0.05; 
        }

        if (iteration == 180) {
            speed = -2;
        }

        if (iteration == 200) {
            speed = 2;
        }

        if (iteration == 220) {
            iteration = 0;
            speed = 0;
        }

        g_a0.setRPS(speed);
        g_a1.setRPS(speed);
        g_b0.setRPS(speed);
        g_b1.setRPS(speed);
        
        iteration++;
#endif
    }

    if (Serial.read() == 0xFF)
    {
        delay(2);

        int64_t speedL = 0;
        speedL |= (int64_t)Serial.read();
        speedL |= (int64_t)Serial.read() << 8;
        speedL |= (int64_t)Serial.read() << 16;
        speedL |= (int64_t)Serial.read() << 24;
        speedL |= (int64_t)Serial.read() << 32;
        speedL |= (int64_t)Serial.read() << 40;
        speedL |= (int64_t)Serial.read() << 48;
        speedL |= (int64_t)Serial.read() << 56;

        int64_t speedR = 0;
        speedR |= (int64_t)Serial.read();
        speedR |= (int64_t)Serial.read() << 8;
        speedR |= (int64_t)Serial.read() << 16;
        speedR |= (int64_t)Serial.read() << 24;
        speedR |= (int64_t)Serial.read() << 32;
        speedR |= (int64_t)Serial.read() << 40;
        speedR |= (int64_t)Serial.read() << 48;
        speedR |= (int64_t)Serial.read() << 56;

        uint8_t parities = Serial.read();
        bool parityL = (parities & 0x01);
        bool parityR = ((parities >> 1) & 0x01);

        if (Serial.read() == 0xFF && checkParity(speedL) != parityL  && checkParity(speedR) != parityR) {
            float rotL = (float)speedL / 10000;
            float rotR = (float)speedR / 10000;

            g_a0.setRPS(rotR);
            g_b0.setRPS(rotR);

            g_a1.setRPS(rotL);
            g_b1.setRPS(rotL);
        }
    }
}
