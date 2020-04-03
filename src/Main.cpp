#include "Hardware.hpp"
#include "Motor.hpp"
#include "MotorPID.hpp"

#include <Arduino.h>

// right side
MotorPID g_a0(PWMA_0, AIN1_0, AIN2_0, AENA_0, AENB_0, false, 30, KP_A0, KI_A0, KD_A0);
MotorPID g_b0(PWMB_0, BIN1_0, BIN2_0, BENA_0, BENB_0, false, 30, KP_B0, KI_B0, KD_B0);

// left side
MotorPID g_a1(PWMA_1, AIN1_1, AIN2_1, AENA_1, AENB_1, true, 30, KP_A1, KI_A1, KD_A1);
MotorPID g_b1(PWMB_1, BIN1_1, BIN2_1, BENA_1, BENB_1, true, 30, KP_B1, KI_B1, KD_B1);

unsigned long g_lastOdomTransTime = 0;

void setup()
{
    g_a0.setRPS(0);
    g_b0.setRPS(0);
    g_a1.setRPS(0);
    g_b1.setRPS(0);

    Serial.begin(115200);
}

void loop()
{
    g_a0.handle();
    g_b0.handle();
    g_a1.handle();
    g_b1.handle();

    if (millis() - g_lastOdomTransTime > 100)
    {
        g_lastOdomTransTime = millis();

        Serial.write(0xFF);

        int64_t rotL = (g_b1.getCurrentRPS() + g_a1.getCurrentRPS()) * 10000 / 2;
        Serial.write((uint8_t)(rotL & 0xFF));
        Serial.write((uint8_t)((rotL >> 8) & 0xFF));
        Serial.write((uint8_t)((rotL >> 16) & 0xFF));
        Serial.write((uint8_t)((rotL >> 24) & 0xFF));
        Serial.write((uint8_t)((rotL >> 32) & 0xFF));
        Serial.write((uint8_t)((rotL >> 40) & 0xFF));
        Serial.write((uint8_t)((rotL >> 48) & 0xFF));
        Serial.write((uint8_t)((rotL >> 56) & 0xFF));

        int64_t rotR = (g_b0.getCurrentRPS() + g_a0.getCurrentRPS()) * 10000 / 2;
        Serial.write((uint8_t)(rotR & 0xFF));
        Serial.write((uint8_t)((rotR >> 8) & 0xFF));
        Serial.write((uint8_t)((rotR >> 16) & 0xFF));
        Serial.write((uint8_t)((rotR >> 24) & 0xFF));
        Serial.write((uint8_t)((rotR >> 32) & 0xFF));
        Serial.write((uint8_t)((rotR >> 40) & 0xFF));
        Serial.write((uint8_t)((rotR >> 48) & 0xFF));
        Serial.write((uint8_t)((rotR >> 56) & 0xFF));

        Serial.flush();
    }

    if (Serial.read() == 0xff)
    {
        delay(2);

        int64_t speedL = 0;
        speedL |= (int64_t) Serial.read();
        speedL |= (int64_t) Serial.read() << 8;
        speedL |= (int64_t) Serial.read() << 16;
        speedL |= (int64_t) Serial.read() << 24;
        speedL |= (int64_t) Serial.read() << 32;
        speedL |= (int64_t) Serial.read() << 40;
        speedL |= (int64_t) Serial.read() << 48;
        speedL |= (int64_t) Serial.read() << 56;

        int64_t speedR = 0;
        speedR |= (int64_t) Serial.read();
        speedR |= (int64_t) Serial.read() << 8;
        speedR |= (int64_t) Serial.read() << 16;
        speedR |= (int64_t) Serial.read() << 24;
        speedR |= (int64_t) Serial.read() << 32;
        speedR |= (int64_t) Serial.read() << 40;
        speedR |= (int64_t) Serial.read() << 48;
        speedR |= (int64_t) Serial.read() << 56;

        float rotL = (float) speedL / 10000;
        float rotR = (float) speedR / 10000;

        g_a0.setRPS(rotR);
        g_b0.setRPS(rotR);

        g_a1.setRPS(rotL);
        g_b1.setRPS(rotL);
    }
}
