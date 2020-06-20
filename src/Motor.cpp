#include "Motor.hpp"

Motor::Motor(uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t ena, uint8_t enb, bool invert)
    : m_invert(invert), m_pwm(pwm), m_in1(in1), m_in2(in2), m_ena(ena), m_enb(enb)
{
    pinMode(pwm, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(ena, INPUT);
    pinMode(enb, INPUT);

    m_pulses = 0;
    setPWM(0);
}

void Motor::setPWM(int16_t speed)
{
    m_speed_pwm = speed;

    applySpeed();
}

int16_t Motor::getPWM() const
{
    return m_speed_pwm;
}

void Motor::handle()
{
    bool envA = analogRead(m_ena) > 511;
    bool envB = analogRead(m_enb) > 511;

    if (envA != m_prevEncA)
    {
        if (envA)
        {
            // rising edge
            if (m_prevEncB)
            {
                addPulse();
            }
            else
            {
                subPulse();
            }
        }
        else
        {
            // falling edge
            if (m_prevEncB)
            {
                subPulse();
            }
            else
            {
                addPulse();
            }
        }
    }

    if (envB != m_prevEncB)
    {
        if (envB)
        {
            // rising edge
            if (m_prevEncA)
            {
                subPulse();
            }
            else
            {
                addPulse();
            }
        }
        else
        {
            // falling edge
            if (m_prevEncA)
            {
                addPulse();
            }
            else
            {
                subPulse();
            }
        }
    }

    m_prevEncA = envA;
    m_prevEncB = envB;
}

void Motor::resetPulses()
{
    m_pulses = 0;
}

int64_t Motor::getPulses() const
{
    return m_pulses;
}

void Motor::applySpeed()
{
    if (m_speed_pwm == 0)
    {
        digitalWrite(m_in1, HIGH);
        digitalWrite(m_in2, HIGH);
        analogWrite(m_pwm, 0);
    }
    else if (m_speed_pwm < 0)
    {
        if (m_invert)
        {
            digitalWrite(m_in1, HIGH);
            digitalWrite(m_in2, LOW);
            analogWrite(m_pwm, -m_speed_pwm);
        }
        else
        {
            digitalWrite(m_in1, LOW);
            digitalWrite(m_in2, HIGH);
            analogWrite(m_pwm, -m_speed_pwm);
        }
    }
    else
    {
        if (m_invert)
        {
            digitalWrite(m_in1, LOW);
            digitalWrite(m_in2, HIGH);
            analogWrite(m_pwm, m_speed_pwm);
        }
        else
        {
            digitalWrite(m_in1, HIGH);
            digitalWrite(m_in2, LOW);
            analogWrite(m_pwm, m_speed_pwm);
        }
    }
}

void Motor::addPulse()
{
    if (!m_invert)
    {
        m_pulses++;
    }
    else
    {
        m_pulses--;
    }

    if (m_pulses > 10000 || m_pulses < -10000)
    {
        resetPulses();
    }
}

void Motor::subPulse()
{
    if (!m_invert)
    {
        m_pulses--;
    }
    else
    {
        m_pulses++;
    }

    if (m_pulses > 10000 || m_pulses < -10000)
    {
        resetPulses();
    }
}
