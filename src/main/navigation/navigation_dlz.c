

#include "msp/msp.h"
#include "msp/msp_protocol_v2_sensor_msg.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "navigation/navigation_dlz.h"
#include "common/log.h"

#define UPDATE_TIMEOUT_MS 1000  // if no update in this time, DLZ is considered lost
#define MAX_DIST 250

static volatile bool isNewDataReady = false;
static uint32_t lastDataRx = 0;
static navDlzData_t _mspData = {0};
static navDlzData_t navDlzData = {0};

static float conditionedNavDlzPosX = 0.0f;
static float conditionedNavDlzPosY = 0.0f;
static float conditionedNavDlzPosZ = 0.0f;
static float conditionedNavDlzConfidence = 0.0f;

/* forward declaration so callers before the definition see the correct type */
static bool copyMspSensorData(navDlzData_t * data);


void navigationDlzInit(void) {

    navDlzData.nedPx = 0;
    navDlzData.nedPy = 0;
    navDlzData.nedPz = 0;
    navDlzData.confidence = 0;

    conditionedNavDlzPosX = 0.0f;
    conditionedNavDlzPosY = 0.0f;
    conditionedNavDlzPosZ = 0.0f;
    conditionedNavDlzConfidence = 0.0f;

    isNewDataReady = false;

}


void navigationDlzUpdate(void) {


    if (millis() - lastDataRx > UPDATE_TIMEOUT_MS) {

        navDlzData.nedPx = 0;
        navDlzData.nedPy = 0;
        navDlzData.nedPz = 0;
        navDlzData.confidence = 0;

        conditionedNavDlzPosX = 0.0f;
        conditionedNavDlzPosY = 0.0f;
        conditionedNavDlzPosZ = 0.0f;
        conditionedNavDlzConfidence = 0.0f;

        isNewDataReady = false;
        return;
    }

    const bool status = copyMspSensorData(&navDlzData);

    if (status) {
        conditionedNavDlzPosX = (float)constrainf(navDlzData.nedPx, -MAX_DIST, MAX_DIST);
        conditionedNavDlzPosY = (float)constrainf(navDlzData.nedPy, -MAX_DIST, MAX_DIST);
        conditionedNavDlzPosZ = (float)constrainf(navDlzData.nedPz, 0, 10000); // max 100m altitude
        conditionedNavDlzConfidence = (float)constrainf(navDlzData.confidence, 0, 1000) / 1000.0f;
    }

}


void navigationDlzReceiveNewData(uint8_t *bufferPtr, unsigned int dataSize)
{

    if(dataSize != sizeof(mspSensorDlz_t)) {
        return;
    }

    const mspSensorDlz_t * pkt = (const mspSensorDlz_t *)bufferPtr;

    _mspData.nedPx = pkt->nedPx; 
    _mspData.nedPy = pkt->nedPy; 
    _mspData.nedPz = pkt->nedPz;
    _mspData.confidence = pkt->confidence;

    //LOG_DEBUG(SYSTEM, "Received DLZ data: nedPx=%d, nedPy=%d, nedPz=%d, confidence=%d", 
    //    _mspData.nedPx, _mspData.nedPy, _mspData.nedPz, _mspData.confidence);

    isNewDataReady = true;
    lastDataRx = millis();

}

static bool copyMspSensorData(navDlzData_t * data) {



    if (isNewDataReady) {
        //LOG_DEBUG(SYSTEM, "Copying DLZ data to output struct");
        *data = _mspData;
        isNewDataReady = false;
        return true;
    }

    return false;

}

float navigatioDlzGetNedPx(void) {
    return conditionedNavDlzPosX;
}

float navigatioDlzGetNedPy(void) {
    return conditionedNavDlzPosY;
}

float navigatioDlzGetNedPz(void) {
    return conditionedNavDlzPosZ;
}

float navigatioDlzGetConfidence(void) {
    return conditionedNavDlzConfidence;
}