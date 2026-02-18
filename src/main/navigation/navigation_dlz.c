#include "common/maths.h"
#include "common/vector.h"
#include "drivers/time.h"
#include "common/log.h"

#include "navigation/navigation_dlz.h"

#define UPDATE_TIMEOUT_MS 1000  // if no update in this time, DLZ is considered lost
#define MAX_DIST 250.0f  // max 2.5m
#define MAX_ALT 10000.0f // max 100 m

static volatile bool isNewDataReady = false;
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

void navigationDlzInit(void) {

    navDlzData.px = 0.0f;
    navDlzData.py = 0.0f;
    navDlzData.pz = 0.0f;
    navDlzData.confidence = 0.0f;
    navDlzData.timestamp_ms = 0;

    isNewDataReady = false;
    //setSkyvisFlag(0);

}


void navigationDlzUpdate(float posErrorX, float posErrorY) {


    if (millis() - navDlzData.timestamp_ms > UPDATE_TIMEOUT_MS) {

        navDlzData.px = 0.0f;
        navDlzData.py = 0.0f;
        navDlzData.pz = 0.0f;
        navDlzData.confidence = 0.0f;

        conditionedNavDlzPosX = 0.0f;
        conditionedNavDlzPosY = 0.0f;
        conditionedNavDlzPosZ = 0.0f;
        conditionedNavDlzConfidence = 0.0f;

        isNewDataReady = false;
        //setSkyvisFlag(10); // No response
        return;
    }


    if (isNewDataReady) {

        fpVector3_t pos = {
            .x = constrainf(navDlzData.px, -MAX_DIST, MAX_DIST),
            .y = constrainf(navDlzData.py, -MAX_DIST, MAX_DIST),
            .z = constrainf(navDlzData.pz, -MAX_ALT, MAX_ALT)
        };

        imuTransformVectorBodyToEarth(&pos);

        conditionedNavDlzPosX = -pos.x; // So that position regulation works correct
        conditionedNavDlzPosY = -pos.y; // So that position regulation works correct
        conditionedNavDlzPosZ = pos.z;
        conditionedNavDlzConfidence = constrainf(navDlzData.confidence, 0.0f, 1.0f);

        // For telemetry 
        if (conditionedNavDlzConfidence > 0.95f) {
            //setSkyvisFlag(30); // Tag detected
        } else {
            //setSkyvisFlag(20); // Comms ok, but no tag detected
        }
    }



}




void navigationDlzReceiveNewData(const float px,
                                 const float py,
                                 const float pz,
                                 const float confidence) {

    navDlzData.px = px;
    navDlzData.py = py;
    navDlzData.pz = pz;
    navDlzData.confidence = confidence;
    navDlzData.timestamp_ms = millis();
    isNewDataReady = true;

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