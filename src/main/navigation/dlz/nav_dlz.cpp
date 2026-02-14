
#include "nav_dlz.hpp"
#include <stdio.h>
#include <cstring>

extern "C" {

#include "common/maths.h"
#include "common/log.h"
#include "programming/logic_condition.h"

#include "navigation/navigation_pos_estimator_private.h"

// FIX ME REMOVE
#include "rx/rx.h"
#include "fc/runtime_config.h"
#include "navigation/navigation_private.h"

}

#define UPDATE_TIMEOUT_MS 1000  // if no update in this time, DLZ is considered lost

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

    m_weighedNedPosX = 0.0f;
    m_weighedNedPosY = 0.0f;
}        


void Navigation::readSkyvisData(const uint8_t* bufferPtr, 
                                unsigned int dataSize) {

    if(dataSize != sizeof(mspSensorSkyvis_t)) {
        LOG_ERROR(SYSTEM, "mspSkyvisReceiveNewData: invalid data size %d", dataSize);
        return;
    }

    memcpy(&m_skyvisData, bufferPtr, sizeof(mspSensorSkyvis_t));

    m_lastMspRxTime = millis();
}   


void Navigation::update() {


    if(!ARMING_FLAG(SIMULATOR_MODE_SITL) && 
        (millis() - m_lastMspRxTime > UPDATE_TIMEOUT_MS)) {
        //LOG_INFO(SYSTEM, "INAV: DLZ Timeout! Time=%u", (unsigned)millis());
        
        // Disable DLZ guidance
        m_nedPosX = 0.0f;
        m_nedPosY = 0.0f;
        m_nedVelX = 0.0f;
        m_nedVelY = 0.0f;
        m_fade = 0.0f;
        m_weighedNedPosX = 0.0f;
        m_weighedNedPosY = 0.0f;
        return;
    }


    const float dt = MS2S(millis() - m_lastUpdateTime);
    const float fade = constrainf(
        (float)m_skyvisData.confidence / 1000.0f, 0.0f, 1.0f);

    if (!isfinite(fade)) {
        m_fade = 0.0f;
    } else {
        m_fade = fade;
    }


    const float WEIGHT = 1.0f; 

    m_weighedNedPosX = m_skyvisData.nedPosX * WEIGHT;
    m_weighedNedPosY = m_skyvisData.nedPosY * WEIGHT;


    m_lastUpdateTime = millis();
}


const float Navigation::getNedPosX() const { return m_nedPosX; }

const float Navigation::getNedPosY() const { return m_nedPosY; }

const float Navigation::getWeighedNedPosX() const { return m_weighedNedPosX; }

const float Navigation::getWeighedNedPosY() const { return m_weighedNedPosY; }

const float Navigation::getNedVelX() const { return m_nedVelX; }

const float Navigation::getNedVelY() const { return m_nedVelY; }

const float Navigation::getFade() const { return m_fade; }


