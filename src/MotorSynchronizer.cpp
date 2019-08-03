#include "MotorSynchronizer.hpp"

MotorSynchronizer::MotorSynchronizer(const uint8_t nMotors, Motor** motors, uint32_t numMillis) 
    : m_nMotors(nMotors), m_motors(motors), m_numMillis(numMillis) {
    
    m_nextSync = 0;
    m_lastPulses = new int64_t[nMotors];
    for (int i = 0; i < nMotors; i++) {
        m_lastPulses[i] = 0;
    }
}

bool MotorSynchronizer::synchronize() {
    if (millis() > m_nextSync) {
        float diff;
        for (int i = 0; i < m_nMotors; i++) {
            for (int j = 0; j < m_nMotors; j++) {
                if (m_motors[i]->getSpeed() == m_motors[j]->getSpeed()) {
                    diff = (m_motors[i]->getSpeed() < 0 ? -1 : 1) * ((m_motors[i]->getPulses() - m_lastPulses[i]) - (m_motors[j]->getPulses() - m_lastPulses[j])) * 0.001f;
                    m_motors[i]->setSpeedMultiplicator(m_motors[i]->getSpeedMultiplicator() - diff);
                }
            }
        }


        for (int i = 0; i < m_nMotors; i++) {
            m_lastPulses[i] = m_motors[i]->getPulses();
        }
        
        m_nextSync = millis() + m_numMillis;

        return true;
    }
    return false;
}