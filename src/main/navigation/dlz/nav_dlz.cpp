
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
#define MAX_POS_CMD_METERS 20.0f

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


//    if(!ARMING_FLAG(SIMULATOR_MODE_SITL) && 
//        (millis() - m_lastMspRxTime > UPDATE_TIMEOUT_MS)) {
//        //LOG_INFO(SYSTEM, "INAV: DLZ Timeout! Time=%u", (unsigned)millis());
//        
//        // Disable DLZ guidance
//        m_nedPosX = 0.0f;
//        m_nedPosY = 0.0f;
//        m_nedVelX = 0.0f;
//        m_nedVelY = 0.0f;
//        m_fade = 0.0f;
//
//        m_weighedNedPosX = 0.0f;
//        m_weighedNedPosY = 0.0f;
//        return;
//    }


    const float dt = MS2S(millis() - m_lastUpdateTime);
    const float fade = constrainf(
        (float)m_skyvisData.confidence / 1000.0f, 0.0f, 1.0f);

    if (!isfinite(fade)) {
        m_fade = 0.0f;
    } else {
        m_fade = fade;
    }

    //m_nedPosX = (float)m_skyvisData.nedPosX;
    //m_nedPosY = (float)m_skyvisData.nedPosY;
    //m_nedVelX = (float)m_skyvisData.nedVelX;
    //m_nedVelY = (float)m_skyvisData.nedVelY;



    // FIX ME REMOVE HERE ------------------------------------------------- //

    if (logicConditionGetValue(DLZ_LOGIC_COND_ID) != 0) {

        //LOG_DEBUG(SYSTEM, "DLZ.Ch10: %f", m_nedPosX);
        //LOG_DEBUG(SYSTEM, "DLZ.Ch11: %f", m_nedPosY);
        //LOG_DEBUG(SYSTEM, "DLZ.Z: dt %f", dt);

        m_fade = 0.95f;

    } else {

        m_fade = 0.0f;

    }




    // Simulate offset coordinates for testing
    // Read primary RC channels (processed values). Channels are 0-based.
    int16_t rcX = rxGetChannelValue(9);  
    int16_t rcY = rxGetChannelValue(10);  

    // Implement deadband
    if (ABS(rcX - 1500) < 20) {
        rcX = 1500;
    }

    if (ABS(rcY - 1500) < 20) {
        rcY = 1500;
    }

    // Scale RC input to max position command, to -1 .. 1
    float fwd = ((float)rcX - 1500.0f) / 500.0f;
    float right = ((float)rcY - 1500.0f) / 500.0f;


    // Body (FR) → World (NE)
    m_nedPosX = fwd * posControl.actualState.cosYaw
                    - right   * posControl.actualState.sinYaw;

    m_nedPosY = fwd * posControl.actualState.sinYaw
                    + right   * posControl.actualState.cosYaw;


    const float WEIGHT = 500.0f; // max 500 cm correction

    m_weighedNedPosX = m_nedPosX * WEIGHT;
    m_weighedNedPosY = m_nedPosY * WEIGHT;


    // REMOVE UP TO HERE ------------------------------------------------- //


//    if ( 
//        (FLIGHT_MODE(NAV_POSHOLD_MODE) || FLIGHT_MODE(NAV_WP_MODE)) &&
//        logicConditionGetValue(DLZ_LOGIC_COND_ID) != 0
//    ) {
//
//        LOG_DEBUG(SYSTEM, "DLZ.Ch10: %f", m_nedPosX);
//        LOG_DEBUG(SYSTEM, "DLZ.Ch11: %f", m_nedPosY);
//        LOG_DEBUG(SYSTEM, "DLZ.Z: dt %f", dt);
//
//    }

    //LOG_INFO(SYSTEM, "INAV: Skyvis.NedPosX %f", m_nedPosX);
    //LOG_INFO(SYSTEM, "INAV: Skyvis.NedPosY %f", m_nedPosY);
    //LOG_INFO(SYSTEM, "INAV: Skyvis.NedVelX %f", m_nedVelX);  
    //LOG_INFO(SYSTEM, "INAV: Skyvis.NedVelY %f", m_nedVelY);
    //LOG_INFO(SYSTEM, "INAV: Skyvis.Confidence %f, raw: %d", m_fade, m_skyvisData.confidence);


    m_lastUpdateTime = millis();
}


const float Navigation::getNedPosX() const { return m_nedPosX; }

const float Navigation::getNedPosY() const { return m_nedPosY; }

const float Navigation::getWeighedNedPosX() const { return m_weighedNedPosX; }

const float Navigation::getWeighedNedPosY() const { return m_weighedNedPosY; }

const float Navigation::getNedVelX() const { return m_nedVelX; }

const float Navigation::getNedVelY() const { return m_nedVelY; }

const float Navigation::getFade() const { return m_fade; }


