

#include "msp/msp.h"
#include "msp/msp_protocol_v2_sensor_msg.h"

#include "navigation/navigation_dlz.h"

#define UPDATE_TIMEOUT_MS 1000  // if no update in this time, DLZ is considered lost

bool isNewDataReady = false;
bool isDataBeingRead = false;

uint32_t lastDataRx = 0;
navDlzData_t _navDlzData;


float navDlzPosX = 0.0f;
float navDlzPosY = 0.0f;
float navDlzConfidence = 0.0f;


void navigationDlzInit(void) {

    navDlzPosX = 0.0f;
    navDlzPosY = 0.0f;
    navDlzConfidence = 0.0f;
    isNewDataReady = false;
    isDataBeingRead = false;

}


void navigationDlzUpdate(void) {


    if (millis() - lastDataRx > UPDATE_TIMEOUT_MS) {
        navDlzPosX = 0.0f;
        navDlzPosY = 0.0f;
        navDlzConfidence = 0.0f;
        isNewDataReady = false;
        isDataBeingRead = false;
        return;
    }


    if (isNewDataReady) {
        isDataBeingRead = true;
            navDlzPosX = (float)constrain(_navDlzData.nedPx, -500, 500);
            navDlzPosY = (float)constrain(_navDlzData.nedPy, -500, 500);
            navDlzConfidence = (float)constrain(_navDlzData.confidence, 0, 1000) / 1000.0f;
        isDataBeingRead = false;
    }

    isNewDataReady = false;

}


void navigationDlzReceiveNewData(uint8_t *bufferPtr, unsigned int dataSize)
{

    if(dataSize != sizeof(mspSensorDlz_t)) {
        return;
    }

    const mspSensorDlz_t * pkt = (const mspSensorDlz_t *)bufferPtr;

    if (isDataBeingRead) return;

    _navDlzData.nedPx = pkt->nedPx; 
    _navDlzData.nedPy = pkt->nedPy; 
    _navDlzData.confidence = pkt->confidence;

    isNewDataReady = true;
    lastDataRx = millis();

}

const float navigatioDlzGetNedPx(void) {
    return navDlzPosX;
}

const float navigatioDlzGetNedPy(void) {
    return navDlzPosY;
}

const float navigatioDlzGetConfidence(void) {
    return navDlzConfidence;
}