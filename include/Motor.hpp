#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class Motor {
public:
    Motor(const uint8_t pwm, const uint8_t in1, const uint8_t in2, const uint8_t ena, const uint8_t enb, const bool reverse, const float speedMultiplicator = 1);

    void setSpeed(const int16_t speed);

    uint16_t getSpeed() const;

    void setSpeedMultiplicator(const float speedMultiplicator);

    float getSpeedMultiplicator() const;

    void readPulsesFromSensor();

    void resetPulses();

    int32_t getPulses() const;

private:
    void applySpeed();

    uint8_t m_pwm;
    uint8_t m_in1;
    uint8_t m_in2;

    /// motor encoders
    uint8_t m_ena;
    uint8_t m_enb;

    int16_t m_speed;

    /// rotation pulses
    int32_t m_pulses;

    bool m_prevEncoderState;

    bool m_reverse;

    float m_speedMultiplicator;
};

#endif