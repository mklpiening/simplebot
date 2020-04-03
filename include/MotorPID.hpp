#ifndef MOTOR_PID_HPP
#define MOTOR_PID_HPP

#include "Motor.hpp"

#include <Arduino.h>

class MotorPID : public Motor
{
  public:
    MotorPID(uint8_t pwm,
             uint8_t in1,
             uint8_t in2,
             uint8_t ena,
             uint8_t enb,
             bool invert,
             unsigned int updateInterval,
             float kp,
             float ki,
             float kd);

    virtual void handle();

    void setRPS(float rps);
    float getRPS();

    float getCurrentRPS();

  private:
    unsigned int m_updateInterval;
    unsigned long m_lastSpeedUpdateTime;

    float m_kp;
    float m_ki;
    float m_kd;

    float m_targetRPS;

    int64_t m_lastPulses;
    float m_lastRpsError;
    float m_rpsErrorIntegral;
    float m_currentRPS;
};

#endif
