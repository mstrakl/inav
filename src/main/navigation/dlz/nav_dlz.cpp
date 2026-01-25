
#include "nav_dlz.hpp"
#include <stdio.h>

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
    m_skyvisData = {0, 0, 0, 0};

    //m_cmdPitchRL.reset();
    //m_cmdRollRL.reset();
}        


void Navigation::readSkyvisData(const uint8_t* bufferPtr, 
                                unsigned int dataSize) {

    if(dataSize != sizeof(mspSensorSkyvis_t)) {
        LOG_ERROR(SYSTEM, "mspSkyvisReceiveNewData: invalid data size %d", dataSize);
        return;
    }

    m_skyvisData = *(const mspSensorSkyvis_t *)bufferPtr;

    m_lastMspRxTime = millis();
}   


void Navigation::update(const float& centimeterPosX, 
                        const float& centimeterPosY,
                        const float& centimeterVelX, 
                        const float& centimeterVelY,
                        const float& navPitchCmd,
                        const float& navRollCmd) {

    if ((millis() - m_lastMspRxTime > UPDATE_TIMEOUT_MS) || 
        (logicConditionGetValue(DLZ_LOGIC_COND_ID) == 0)) {
        m_cmdPitch = navPitchCmd;
        m_cmdRoll = navRollCmd;
        LOG_INFO(SYSTEM, "INAV: DLZ Timeout! Time=%u", (unsigned)millis());
        return;
    }

    const float cmdPitch = (float)m_skyvisData.cmdPitch * (3.14159265f / 180.0f) / 100.0f;   // convert to radians
    const float cmdRoll = (float)m_skyvisData.cmdRoll * (3.14159265f / 180.0f) / 100.0f;     // convert to radians

    const float fade = constrainf((float)m_skyvisData.confidence / 1000.0f, 0.0f, 1.0f);

    m_cmdPitch = fade * cmdPitch + (1.0f - fade) * navPitchCmd;
    m_cmdRoll = fade * cmdRoll + (1.0f - fade) * navRollCmd;

    LOG_INFO(SYSTEM, "INAV: DLZ Update: Time=%u CmdPitchDeg=%f CmdRollDeg=%f VelX=%f VelY=%f",
             (unsigned)millis(), m_cmdPitch * (180.0f / 3.14159265f), m_cmdRoll * (180.0f / 3.14159265f),
             centimeterVelX, centimeterVelY);

    m_lastUpdateTime = millis();
}


float Navigation::getPitchCmd() const {
    return m_cmdPitch;
}

float Navigation::getRollCmd() const {
    return m_cmdRoll;
}


