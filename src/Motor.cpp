#include "Motor.hpp"

Motor::Motor(const uint8_t pwm, const uint8_t in1, const uint8_t in2, const uint8_t ena, const uint8_t enb, const bool reverse, const float speedMultiplicator)
    : m_pwm(pwm), m_in1(in1), m_in2(in2), m_ena(ena), m_enb(enb), m_reverse(reverse), m_speedMultiplicator(speedMultiplicator) {
    
    pinMode(pwm, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(ena, INPUT);
    pinMode(enb, INPUT);
    
    m_pulses = 0;
    setSpeed(0);
}

void Motor::setSpeed(const int16_t speed) {
    if (m_reverse) {
        m_speed = -speed;
    } else {
        m_speed = speed;
    }

    applySpeed();
}

void Motor::setSpeedMultiplicator(const float speedMultiplicator) {
    m_speedMultiplicator = speedMultiplicator;

    applySpeed();
}

float Motor::getSpeedMultiplicator() {
    return m_speedMultiplicator;
}

void Motor::readPulsesFromSensor() {
    if (analogRead(m_ena) > 511) {
        if (m_prevEncoderState == true) {
            if (analogRead(m_enb) < 511) {
                if (m_reverse) {
                    m_pulses++;
                } else {
                    m_pulses--;
                }
            } else {
                if (m_reverse) {
                    m_pulses--;
                } else {
                    m_pulses++;
                }
            }

            Serial.println(m_pulses);

            m_prevEncoderState = false;
        }
    } else {
        if (m_prevEncoderState == false) {
            m_prevEncoderState = true;
        }
    }
}

void Motor::resetPulses() {
    m_pulses = 0;
}

int32_t Motor::getPulses() {
    return m_pulses;
}

void Motor::applySpeed() {
    if (m_speed == 0) {
        digitalWrite(m_in1, HIGH);
        digitalWrite(m_in2, HIGH);
        analogWrite(m_pwm, 0);
    } else if (m_speed < 0) {
        digitalWrite(m_in1, LOW);
        digitalWrite(m_in2, HIGH);
        analogWrite(m_pwm, -m_speedMultiplicator * m_speed);
    } else {
        digitalWrite(m_in1, HIGH);
        digitalWrite(m_in2, LOW);
        analogWrite(m_pwm, m_speedMultiplicator * m_speed);
    }
}