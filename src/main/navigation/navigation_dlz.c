

#include "msp/msp.h"
#include "msp/msp_protocol_v2_sensor_msg.h"
#include "common/maths.h"
#include "common/vector.h"
#include "drivers/time.h"
#include "navigation/navigation_dlz.h"
#include "sensors/sensors.h"
#include "common/log.h"
#include "telemetry/mavlink.h"

#define UPDATE_TIMEOUT_MS 1000  // if no update in this time, DLZ is considered lost
#define MAX_DIST 250
#define USE_DLZ_COORDS_BODY true

static volatile bool isNewDataReady = false;
static uint32_t lastDataRx = 0;
static navDlzData_t _mspData = {0};
static navDlzData_t navDlzData = {0};

static float conditionedNavDlzPosX = 0.0f;
static float conditionedNavDlzPosY = 0.0f;
static float conditionedNavDlzPosZ = 0.0f;
static float conditionedNavDlzConfidence = 0.0f;

// Skyvis status flag
//  UNDEFINED = 0
//  NO_RESPONSE = 10
//  COMMS_OK = 20
//  TAG_DETECTED = 30


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

    setSkyvisFlag(0);

}


void navigationDlzUpdate(void) {

/*
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

        setSkyvisFlag(10); // No response
        return;
    }
*/
    if (true) {

            //fpVector3_t pos = {
            //    .x = (float)constrainf(navDlzData.nedPx, -MAX_DIST, MAX_DIST),
            //    .y = (float)constrainf(navDlzData.nedPy, -MAX_DIST, MAX_DIST),
            //    .z = (float)constrainf(navDlzData.nedPz, 0, 10000)
            //};

            fpVector3_t pos = {
                .x = mavlinkDlzData.nedPx,
                .y = mavlinkDlzData.nedPy,
                .z = mavlinkDlzData.nedPz
            };

            imuTransformVectorBodyToEarth(&pos);

            conditionedNavDlzPosX = -pos.x; // So that position regulation works correct
            conditionedNavDlzPosY = -pos.y; // So that position regulation works correct
            conditionedNavDlzPosZ = pos.z;

        } 

        conditionedNavDlzConfidence = mavlinkDlzData.confidence;

        // For telemetry 
        if (conditionedNavDlzConfidence > 0.95f) {
            setSkyvisFlag(30); // Tag detected
        } else {
            setSkyvisFlag(20); // Comms ok, but no tag detected
        }

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