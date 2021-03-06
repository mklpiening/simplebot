#include "MotorPID.hpp"

#include "Hardware.hpp"

MotorPID::MotorPID(uint8_t pwm,
                   uint8_t in1,
                   uint8_t in2,
                   uint8_t ena,
                   uint8_t enb,
                   bool invert,
                   unsigned int updateInterval)
    : Motor(pwm, in1, in2, ena, enb, invert), m_updateInterval(updateInterval),
      m_lastSpeedUpdateTime(0), m_rpsToPwm(0), m_kp(0), m_ki(0), m_kd(0), m_lastPulses(0), m_lastRpsError(0),
      m_rpsErrorIntegral(0), m_currentRPS(0)
{
}

void MotorPID::handle()
{
    Motor::handle();

    if (millis() - m_lastSpeedUpdateTime > m_updateInterval)
    {
        m_lastSpeedUpdateTime = millis();

        if (abs(getPulses() - m_lastPulses) > 1000)
        {
            m_lastPulses = getPulses();
            return;
        }

        float currentRPS = (float)(getPulses() - m_lastPulses) * 1000.0
                           / (float)(PULSES_PER_ROTATION * m_updateInterval);
        // m_currentRPS = 0.01 * m_currentRPS + 0.99 * currentRPS;
        m_currentRPS = currentRPS;

        float rpsError = m_targetRPS - m_currentRPS;
        m_rpsErrorIntegral += rpsError * m_updateInterval;

        // limit integral to one rotation in one second
        m_rpsErrorIntegral = min(m_rpsErrorIntegral, 1000.0);
        m_rpsErrorIntegral = max(m_rpsErrorIntegral, -1000.0);

        int16_t pwmValue = m_rpsToPwm * m_targetRPS + m_kp * rpsError + m_ki * m_rpsErrorIntegral
                           + m_kd * (rpsError - m_lastRpsError) * 1000.0 / m_updateInterval;

        if (abs(pwmValue) < 30)
        {
            pwmValue = 0;
        }

        pwmValue = max(pwmValue, -255);
        pwmValue = min(pwmValue, 255);

        // setPWM(pwmValue * 0.6 + getPWM() * 0.4);
        setPWM(pwmValue);

        m_lastPulses   = getPulses();
        m_lastRpsError = rpsError;
    }
}

void MotorPID::setRPS(float rps)
{
    m_targetRPS = rps;
}

float MotorPID::getRPS()
{
    return m_targetRPS;
}

float MotorPID::getCurrentRPS()
{
    return m_currentRPS;
}

void MotorPID::setPIDKoeffs(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void MotorPID::setRPStoPWMFactor(float k)
{
    m_rpsToPwm = k;
}
