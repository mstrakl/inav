
#include "nav_dlz.hpp"
#include <stdio.h>
#include <cstring>

extern "C" {
#include "common/maths.h"
#include "common/log.h"
#include "programming/logic_condition.h"
}

#define UPDATE_TIMEOUT_MS 1000  // if no update in this time, DLZ is considered lost
#define MAX_POS_CMD_METERS 20.0f
#define DLZ_LOGIC_COND_ID 50  // Logic condition ID to check if DLZ is active

using namespace AdumDlz;


Navigation::Navigation() {
    reset();
}


void Navigation::reset() {
    m_lastMspRxTime = 0;
    m_lastUpdateTime = 0;
    m_skyvisData = {0, 0, 0, 0, 0};

    m_nedPosX = 0.0f;   
    m_nedPosY = 0.0f;
    m_nedVelX = 0.0f;
    m_nedVelY = 0.0f;
    m_fade = 0.0f;
}        


void Navigation::readSkyvisData(const uint8_t* bufferPtr, 
                                unsigned int dataSize) {

    if(dataSize != sizeof(mspSensorSkyvis_t)) {
        LOG_ERROR(SYSTEM, "mspSkyvisReceiveNewData: invalid data size %d", dataSize);
        LOG_DEBUG(SYSTEM, "mspSkyvisReceiveNewData: invalid data size %d", dataSize);
        return;
    }

    memcpy(&m_skyvisData, bufferPtr, sizeof(mspSensorSkyvis_t));

    m_lastMspRxTime = millis();
}   


void Navigation::update() {

    if ((millis() - m_lastMspRxTime > UPDATE_TIMEOUT_MS) || 
        (logicConditionGetValue(DLZ_LOGIC_COND_ID) == 0)) {
        LOG_INFO(SYSTEM, "INAV: DLZ Timeout! Time=%u", (unsigned)millis());
        
        // Disable DLZ guidance
        m_nedPosX = 0.0f;
        m_nedPosY = 0.0f;
        m_nedVelX = 0.0f;
        m_nedVelY = 0.0f;
        m_fade = 0.0f;
        return;
    }


    m_fade = constrainf((float)m_skyvisData.confidence / 1000.0f, 0.0f, 1.0f);
    m_nedPosX = (float)m_skyvisData.nedPosX;
    m_nedPosY = (float)m_skyvisData.nedPosY;
    m_nedVelX = (float)m_skyvisData.nedVelX;
    m_nedVelY = (float)m_skyvisData.nedVelY;


    //LOG_INFO(SYSTEM, "INAV: Skyvis.NedPosX %f", m_nedPosX);
    //LOG_INFO(SYSTEM, "INAV: Skyvis.NedPosY %f", m_nedPosY);
    //LOG_INFO(SYSTEM, "INAV: Skyvis.NedVelX %f", m_nedVelX);  
    //LOG_INFO(SYSTEM, "INAV: Skyvis.NedVelY %f", m_nedVelY);
    //LOG_INFO(SYSTEM, "INAV: Skyvis.Confidence %f, raw: %d", m_fade, m_skyvisData.confidence);


    m_lastUpdateTime = millis();
}


const float Navigation::getNedPosX() const { return m_nedPosX; }

const float Navigation::getNedPosY() const { return m_nedPosY; }

const float Navigation::getNedVelX() const { return m_nedVelX; }

const float Navigation::getNedVelY() const { return m_nedVelY; }

const float Navigation::getFade() const { return m_fade; }


