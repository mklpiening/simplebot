#include "MotorSynchronizer.hpp"

MotorSynchronizer::MotorSynchronizer(const uint8_t nMotors, Motor** motors, uint32_t numMillis) 
    : m_nMotors(nMotors), m_motors(motors), m_numMillis(numMillis) {
    
    m_nextSync = 0;
}

bool MotorSynchronizer::synchronize() {
    if (millis() > m_nextSync) {
        float diff;
        for (int i = 0; i < m_nMotors; i++) {
            for (int j = 0; j < m_nMotors; j++) {
                if (m_motors[i]->getSpeed() == m_motors[j]->getSpeed()) {
                    diff = (m_motors[i]->getPulses() - m_motors[j]->getPulses()) * 0.0001f;
                    m_motors[i]->setSpeedMultiplicator(m_motors[i]->getSpeedMultiplicator() - diff);
                }
            }
        }
        
        m_nextSync = millis() + m_numMillis;

        return true;
    }
    return false;
}