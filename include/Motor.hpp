#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class Motor
{
  public:
    Motor(uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t ena, uint8_t enb, bool invert);

    void setPWM(int16_t speed);

    int16_t getPWM() const;

    virtual void handle();

    void resetPulses();

    int64_t getPulses() const;

  protected:
    /// true if the motor direction is inverted
    bool m_invert;

  private:
    void applySpeed();

    void addPulse();
    void subPulse();

    /// motor pins
    uint8_t m_pwm;
    uint8_t m_in1;
    uint8_t m_in2;

    /// motor encoders
    uint8_t m_ena;
    uint8_t m_enb;

    /// current pwm value used for the motor speed
    int16_t m_speed_pwm;

    /// rotation pulses
    int64_t m_pulses;

    /// previous encoder pulse state (1 or 0)
    bool m_prevEncA = true;
    bool m_prevEncB = true;
};

#endif
