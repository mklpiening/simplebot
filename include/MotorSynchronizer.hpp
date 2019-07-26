#ifndef MOTOR_SYNCHRONIZER_HPP
#define MOTOR_SYNCHRONIZER_HPP

#include <Arduino.h>

#include "Motor.hpp"

class MotorSynchronizer {
public:
    MotorSynchronizer(const uint8_t nMotors, Motor** motors, uint32_t numMillis = 500);

    bool synchronize();

private:
    uint8_t m_nMotors;
    Motor** m_motors;

    uint32_t m_numMillis;
    unsigned long m_nextSync;
};

#endif