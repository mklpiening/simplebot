#include "Hardware.hpp"
#include "Motor.hpp"
#include "MotorPID.hpp"

#include <Arduino.h>

// right side
MotorPID g_fr(PWMA_0, AIN1_0, AIN2_0, AENA_0, AENB_0, false, 80);
MotorPID g_br(PWMB_0, BIN1_0, BIN2_0, BENA_0, BENB_0, false, 80);

// left side
MotorPID g_fl(PWMA_1, AIN1_1, AIN2_1, AENA_1, AENB_1, true, 80);
MotorPID g_bl(PWMB_1, BIN1_1, BIN2_1, BENA_1, BENB_1, true, 80);

unsigned long g_lastOdomTransTime = 0;

float g_lastRotFL = 0;
float g_lastRotRL = 0;
float g_lastRotFR = 0;
float g_lastRotRR = 0;

bool g_motor_params_received = false;

bool checkParity(int64_t data)
{
    int numOnes = 0;
    for (int i = 0; i < 64; i++)
    {
        if (data & (1 << i))
        {
            numOnes++;
        }
    }

    return numOnes % 2 == 0;
}

void setup()
{
    g_fr.setRPS(0);
    g_br.setRPS(0);
    g_fl.setRPS(0);
    g_bl.setRPS(0);

    Serial.begin(115200);

    pinMode(13, OUTPUT);
}

void loop()
{
    if (g_motor_params_received)
    {
        g_fr.handle();
        g_br.handle();
        g_fl.handle();
        g_bl.handle();
    }

    if (millis() - g_lastOdomTransTime > 200)
    {
        Serial.write(0xFF);

        float rotFL    = (float)g_fl.getPulses() / PULSES_PER_ROTATION;
        int16_t dRotFL = (rotFL - g_lastRotFL) * 10000;
        g_lastRotFL    = rotFL;
        Serial.write((uint8_t)(dRotFL & 0xFF));
        Serial.write((uint8_t)((dRotFL >> 8) & 0xFF));

        float rotRL    = (float)g_bl.getPulses() / PULSES_PER_ROTATION;
        int16_t dRotRL = (rotRL - g_lastRotRL) * 10000;
        g_lastRotRL    = rotRL;
        Serial.write((uint8_t)(dRotRL & 0xFF));
        Serial.write((uint8_t)((dRotRL >> 8) & 0xFF));

        float rotFR    = (float)g_fr.getPulses() / PULSES_PER_ROTATION;
        int16_t dRotFR = (rotFR - g_lastRotFR) * 10000;
        g_lastRotFR    = rotFR;
        Serial.write((uint8_t)(dRotFR & 0xFF));
        Serial.write((uint8_t)((dRotFR >> 8) & 0xFF));

        float rotRR    = (float)g_br.getPulses() / PULSES_PER_ROTATION;
        int16_t dRotRR = (rotRR - g_lastRotRR) * 10000;
        g_lastRotRR    = rotRR;
        Serial.write((uint8_t)(dRotRR & 0xFF));
        Serial.write((uint8_t)((dRotRR >> 8) & 0xFF));

        Serial.write((uint8_t)(millis() - g_lastOdomTransTime));

        Serial.flush();

        g_lastOdomTransTime = millis();
    }

    if (Serial.available() >= 2)
    {
        if (Serial.read() == 0xFF)
        {
            uint8_t message_type = Serial.read();
            if (message_type == 0x00)
            {
                // INCOMING SPEED MESSAGE

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
                bool parityL     = (parities & 0x01);
                bool parityR     = ((parities >> 1) & 0x01);

                if (Serial.read() == 0xFF && checkParity(speedL) != parityL
                    && checkParity(speedR) != parityR)
                {
                    float rotL = (float)speedL / 10000.0;
                    float rotR = (float)speedR / 10000.0;

                    g_fr.setRPS(rotR);
                    g_br.setRPS(rotR);

                    g_fl.setRPS(rotL);
                    g_bl.setRPS(rotL);
                }
            }
            else if (message_type == 0x01)
            {
                // INCOMING MOTOR PID PARAMETERS MESSAGE

                int64_t kp, ki, kd, rpsToPwm;

                // Motor Front Right
                while (Serial.available() < 8)
                    ;
                kp = 0;
                kp |= (int64_t)Serial.read();
                kp |= (int64_t)Serial.read() << 8;
                kp |= (int64_t)Serial.read() << 16;
                kp |= (int64_t)Serial.read() << 24;
                kp |= (int64_t)Serial.read() << 32;
                kp |= (int64_t)Serial.read() << 40;
                kp |= (int64_t)Serial.read() << 48;
                kp |= (int64_t)Serial.read() << 56;

                while (Serial.available() < 8)
                    ;
                ki = 0;
                ki |= (int64_t)Serial.read();
                ki |= (int64_t)Serial.read() << 8;
                ki |= (int64_t)Serial.read() << 16;
                ki |= (int64_t)Serial.read() << 24;
                ki |= (int64_t)Serial.read() << 32;
                ki |= (int64_t)Serial.read() << 40;
                ki |= (int64_t)Serial.read() << 48;
                ki |= (int64_t)Serial.read() << 56;

                while (Serial.available() < 8)
                    ;
                kd = 0;
                kd |= (int64_t)Serial.read();
                kd |= (int64_t)Serial.read() << 8;
                kd |= (int64_t)Serial.read() << 16;
                kd |= (int64_t)Serial.read() << 24;
                kd |= (int64_t)Serial.read() << 32;
                kd |= (int64_t)Serial.read() << 40;
                kd |= (int64_t)Serial.read() << 48;
                kd |= (int64_t)Serial.read() << 56;

                g_fr.setPIDKoeffs((float)kp / 10000.0, (float)ki / 10000.0, (float)kd / 10000.0);

                while (Serial.available() < 8)
                    ;
                rpsToPwm = 0;
                rpsToPwm |= (int64_t)Serial.read();
                rpsToPwm |= (int64_t)Serial.read() << 8;
                rpsToPwm |= (int64_t)Serial.read() << 16;
                rpsToPwm |= (int64_t)Serial.read() << 24;
                rpsToPwm |= (int64_t)Serial.read() << 32;
                rpsToPwm |= (int64_t)Serial.read() << 40;
                rpsToPwm |= (int64_t)Serial.read() << 48;
                rpsToPwm |= (int64_t)Serial.read() << 56;

                g_fr.setRPStoPWMFactor((float) rpsToPwm / 10000.0);

                // Motor Back Right
                while (Serial.available() < 8)
                    ;
                kp = 0;
                kp |= (int64_t)Serial.read();
                kp |= (int64_t)Serial.read() << 8;
                kp |= (int64_t)Serial.read() << 16;
                kp |= (int64_t)Serial.read() << 24;
                kp |= (int64_t)Serial.read() << 32;
                kp |= (int64_t)Serial.read() << 40;
                kp |= (int64_t)Serial.read() << 48;
                kp |= (int64_t)Serial.read() << 56;

                while (Serial.available() < 8)
                    ;
                ki = 0;
                ki |= (int64_t)Serial.read();
                ki |= (int64_t)Serial.read() << 8;
                ki |= (int64_t)Serial.read() << 16;
                ki |= (int64_t)Serial.read() << 24;
                ki |= (int64_t)Serial.read() << 32;
                ki |= (int64_t)Serial.read() << 40;
                ki |= (int64_t)Serial.read() << 48;
                ki |= (int64_t)Serial.read() << 56;

                while (Serial.available() < 8)
                    ;
                kd = 0;
                kd |= (int64_t)Serial.read();
                kd |= (int64_t)Serial.read() << 8;
                kd |= (int64_t)Serial.read() << 16;
                kd |= (int64_t)Serial.read() << 24;
                kd |= (int64_t)Serial.read() << 32;
                kd |= (int64_t)Serial.read() << 40;
                kd |= (int64_t)Serial.read() << 48;
                kd |= (int64_t)Serial.read() << 56;

                g_br.setPIDKoeffs((float)kp / 10000.0, (float)ki / 10000.0, (float)kd / 10000.0);

                while (Serial.available() < 8)
                    ;
                rpsToPwm = 0;
                rpsToPwm |= (int64_t)Serial.read();
                rpsToPwm |= (int64_t)Serial.read() << 8;
                rpsToPwm |= (int64_t)Serial.read() << 16;
                rpsToPwm |= (int64_t)Serial.read() << 24;
                rpsToPwm |= (int64_t)Serial.read() << 32;
                rpsToPwm |= (int64_t)Serial.read() << 40;
                rpsToPwm |= (int64_t)Serial.read() << 48;
                rpsToPwm |= (int64_t)Serial.read() << 56;

                g_br.setRPStoPWMFactor((float) rpsToPwm / 10000.0);

                // Motor Front Left
                while (Serial.available() < 8)
                    ;
                kp = 0;
                kp |= (int64_t)Serial.read();
                kp |= (int64_t)Serial.read() << 8;
                kp |= (int64_t)Serial.read() << 16;
                kp |= (int64_t)Serial.read() << 24;
                kp |= (int64_t)Serial.read() << 32;
                kp |= (int64_t)Serial.read() << 40;
                kp |= (int64_t)Serial.read() << 48;
                kp |= (int64_t)Serial.read() << 56;

                while (Serial.available() < 8)
                    ;
                ki = 0;
                ki |= (int64_t)Serial.read();
                ki |= (int64_t)Serial.read() << 8;
                ki |= (int64_t)Serial.read() << 16;
                ki |= (int64_t)Serial.read() << 24;
                ki |= (int64_t)Serial.read() << 32;
                ki |= (int64_t)Serial.read() << 40;
                ki |= (int64_t)Serial.read() << 48;
                ki |= (int64_t)Serial.read() << 56;

                while (Serial.available() < 8)
                    ;
                kd = 0;
                kd |= (int64_t)Serial.read();
                kd |= (int64_t)Serial.read() << 8;
                kd |= (int64_t)Serial.read() << 16;
                kd |= (int64_t)Serial.read() << 24;
                kd |= (int64_t)Serial.read() << 32;
                kd |= (int64_t)Serial.read() << 40;
                kd |= (int64_t)Serial.read() << 48;
                kd |= (int64_t)Serial.read() << 56;

                g_fl.setPIDKoeffs((float)kp / 10000.0, (float)ki / 10000.0, (float)kd / 10000.0);

                while (Serial.available() < 8);
                    ;
                rpsToPwm = 0;
                rpsToPwm |= (int64_t)Serial.read();
                rpsToPwm |= (int64_t)Serial.read() << 8;
                rpsToPwm |= (int64_t)Serial.read() << 16;
                rpsToPwm |= (int64_t)Serial.read() << 24;
                rpsToPwm |= (int64_t)Serial.read() << 32;
                rpsToPwm |= (int64_t)Serial.read() << 40;
                rpsToPwm |= (int64_t)Serial.read() << 48;
                rpsToPwm |= (int64_t)Serial.read() << 56;

                g_fl.setRPStoPWMFactor((float) rpsToPwm / 10000.0);

                // Motor Back Left
                while (Serial.available() < 8)
                    ;
                kp = 0;
                kp |= (int64_t)Serial.read();
                kp |= (int64_t)Serial.read() << 8;
                kp |= (int64_t)Serial.read() << 16;
                kp |= (int64_t)Serial.read() << 24;
                kp |= (int64_t)Serial.read() << 32;
                kp |= (int64_t)Serial.read() << 40;
                kp |= (int64_t)Serial.read() << 48;
                kp |= (int64_t)Serial.read() << 56;

                while (Serial.available() < 8)
                    ;
                ki = 0;
                ki |= (int64_t)Serial.read();
                ki |= (int64_t)Serial.read() << 8;
                ki |= (int64_t)Serial.read() << 16;
                ki |= (int64_t)Serial.read() << 24;
                ki |= (int64_t)Serial.read() << 32;
                ki |= (int64_t)Serial.read() << 40;
                ki |= (int64_t)Serial.read() << 48;
                ki |= (int64_t)Serial.read() << 56;

                while (Serial.available() < 8)
                    ;
                kd = 0;
                kd |= (int64_t)Serial.read();
                kd |= (int64_t)Serial.read() << 8;
                kd |= (int64_t)Serial.read() << 16;
                kd |= (int64_t)Serial.read() << 24;
                kd |= (int64_t)Serial.read() << 32;
                kd |= (int64_t)Serial.read() << 40;
                kd |= (int64_t)Serial.read() << 48;
                kd |= (int64_t)Serial.read() << 56;

                g_bl.setPIDKoeffs((float)kp / 10000.0, (float)ki / 10000.0, (float)kd / 10000.0);

                while (Serial.available() < 8)
                    ;
                rpsToPwm = 0;
                rpsToPwm |= (int64_t)Serial.read();
                rpsToPwm |= (int64_t)Serial.read() << 8;
                rpsToPwm |= (int64_t)Serial.read() << 16;
                rpsToPwm |= (int64_t)Serial.read() << 24;
                rpsToPwm |= (int64_t)Serial.read() << 32;
                rpsToPwm |= (int64_t)Serial.read() << 40;
                rpsToPwm |= (int64_t)Serial.read() << 48;
                rpsToPwm |= (int64_t)Serial.read() << 56;

                g_bl.setRPStoPWMFactor((float) rpsToPwm / 10000.0);

                g_motor_params_received = true;

                if (Serial.read() == 0xFF)
                {
                    // OK
                }
            }
        }
    }
}
