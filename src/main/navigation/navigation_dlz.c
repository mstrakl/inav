
#include "drivers/time.h"

#include "common/maths.h"
#include "common/vector.h"
#include "common/log.h"

#include "flight/imu.h"

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

static float conditionedBiasPosX = 0.0f;
static float conditionedBiasPosY = 0.0f;

static navRateLimiter_t navDlzBiasPosXRateLimiter;
static navRateLimiter_t navDlzBiasPosYRateLimiter;


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

    conditionedNavDlzPosX = 0.0f;
    conditionedNavDlzPosY = 0.0f;
    conditionedNavDlzPosZ = 0.0f;
    conditionedNavDlzConfidence = 0.0f;

    conditionedBiasPosX = 0.0f;
    conditionedBiasPosY = 0.0f;

    // Rate limited to 25cm/s initially
    navRateLimiterInit(&navDlzBiasPosXRateLimiter, 25.0f, 0.0f, millis());
    navRateLimiterInit(&navDlzBiasPosYRateLimiter, 25.0f, 0.0f, millis());

    isNewDataReady = false;
    //setSkyvisFlag(0);

}


void navigationDlzUpdate(const float posErrorX, const float posErrorY) {


    if (millis() - navDlzData.timestamp_ms > UPDATE_TIMEOUT_MS) {

        navDlzData.px = 0.0f;
        navDlzData.py = 0.0f;
        navDlzData.pz = 0.0f;
        navDlzData.confidence = 0.0f;

        //navDlzData.timestamp_ms = 0; // Don't set timestamp to 0, 
                                       // so that we can detect when new 
                                       // data arrives after timeout

        conditionedNavDlzPosX = 0.0f;
        conditionedNavDlzPosY = 0.0f;
        conditionedNavDlzPosZ = 0.0f;
        conditionedNavDlzConfidence = 0.0f;

        conditionedBiasPosX = 0.0f;
        conditionedBiasPosY = 0.0f;

        isNewDataReady = false;
        //setSkyvisFlag(10); // No response
        return;
    }


    if (isNewDataReady) {


        // Read data, and make sure receive time didn't change

        uint32_t dataTimestamp = navDlzData.timestamp_ms;
        fpVector3_t pos;

        pos.x = constrainf(navDlzData.px, -MAX_DIST, MAX_DIST);
        pos.y = constrainf(navDlzData.py, -MAX_DIST, MAX_DIST);
        pos.z = constrainf(navDlzData.pz, -MAX_ALT, MAX_ALT);
        conditionedNavDlzConfidence = constrainf(navDlzData.confidence, 0.0f, 1.0f);

        if (dataTimestamp != navDlzData.timestamp_ms) {

            // Data was updated, let's read it again
            pos.x = constrainf(navDlzData.px, -MAX_DIST, MAX_DIST);
            pos.y = constrainf(navDlzData.py, -MAX_DIST, MAX_DIST);
            pos.z = constrainf(navDlzData.pz, -MAX_ALT, MAX_ALT);
            conditionedNavDlzConfidence = constrainf(navDlzData.confidence, 0.0f, 1.0f);

        }

        isNewDataReady = false; // Mark data as consumed, so that we don't 
                                // use it again until new data arrives


        // Transform to body

        imuTransformVectorBodyToEarth(&pos);

        conditionedNavDlzPosX = -pos.x; // So that position regulation works correct
        conditionedNavDlzPosY = -pos.y; // So that position regulation works correct
        conditionedNavDlzPosZ = pos.z;

        // Calculate bias, rate limited
        conditionedBiasPosX = navRateLimiterUpdate(&navDlzBiasPosXRateLimiter,
                                                   conditionedNavDlzConfidence * (conditionedNavDlzPosX - posErrorX),
                                                   millis());
                                                   
        conditionedBiasPosY = navRateLimiterUpdate(&navDlzBiasPosYRateLimiter,
                                                   conditionedNavDlzConfidence * (conditionedNavDlzPosY - posErrorY),
                                                   millis());

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

    navDlzData.timestamp_ms = millis();
    navDlzData.px = px;
    navDlzData.py = py;
    navDlzData.pz = pz;
    navDlzData.confidence = confidence;
    isNewDataReady = true;

}


// Rate Limiter


void navRateLimiterInit(navRateLimiter_t *sl, 
                        const float rate_per_sec, 
                        const float initial_value, 
                        const uint32_t now_ms) {

    sl->rate_per_sec = rate_per_sec;
    sl->last_output = initial_value;
    sl->last_time_ms = now_ms;
}


float navRateLimiterUpdate(navRateLimiter_t *sl, 
                           const float target, 
                           const uint32_t now_ms) {

    uint32_t dt_ms = now_ms - sl->last_time_ms;
    sl->last_time_ms = now_ms;

    float dt_sec = dt_ms / 1000.0f;
    float max_delta = sl->rate_per_sec * dt_sec;

    float delta = target - sl->last_output;

    if (delta > max_delta)
        delta = max_delta;
    else if (delta < -max_delta)
        delta = -max_delta;

    sl->last_output += delta;

    return sl->last_output;
}


// Getters



float navigationDlzGetBiasPosX(void) {
    return conditionedBiasPosX;
}

float navigationDlzGetBiasPosY(void) {
    return conditionedBiasPosY;
}

float navigationDlzGetNedPz(void) {
    return conditionedNavDlzPosZ;
}

float navigationDlzGetConfidence(void) {
    return conditionedNavDlzConfidence;
}



/*

// Dlz here

    // True when we're on the final LAND waypoint and it has been reached

    bool landingPntReached = false;

    if (FLIGHT_MODE(NAV_WP_MODE) && 
        (posControl.activeWaypointIndex == posControl.waypointCount - 1) &&
        (posControl.wpDistance < 500.0f)) {
            landingPntReached = true;
        }


    bool dlz_allowed = logicConditionGetValue(DLZ_LOGIC_COND_ID) != 0;

    float biasX = 0.0f;
    float biasY = 0.0f;

    //dlzPosCtrlFade = constrainf(dlzPosCtrlFade, 0.0f, 1.0f);

    navigationDlzUpdate();

    if (dlz_allowed) {

        const float fade = navigatioDlzGetConfidence();

        biasX = fade * (navigatioDlzGetNedPx() - posErrorX);
        biasY = fade * (navigatioDlzGetNedPy() - posErrorY);

        if(landingPntReached && navGetCurrentActualPositionAndVelocity()->pos.z < 200.0f && fade < 0.95f) {
            biasX = dlzOldBiasX;
            biasY = dlzOldBiasY;
        } else {
            if (fade > 0.95f) {
                dlzOldBiasX = biasX;
                dlzOldBiasY = biasY;
            }
        }

        posErrorX += biasX;
        posErrorY += biasY;
        
        //if (dlzPosCtrlFade < 1.0f) dlzPosCtrlFade += 0.01f;

    } else {
        dlzOldBiasX = 0.0f;
        dlzOldBiasY = 0.0f;
        //dlzPosCtrlFade = 0.0f;
    }

    if (millis() - lastPrintTime > PRINT_TIME) {
        LOG_DEBUG(SYSTEM, "DLZ: allowed: %d", dlz_allowed);
        LOG_DEBUG(SYSTEM, "DLZ: fin wp reached: %d", landingPntReached);
        LOG_DEBUG(SYSTEM, "DLZ: dist: %f", posControl.wpDistance);
        LOG_DEBUG(SYSTEM, "DLZ: dpx: %f, dpy: %f, dpz: %f", navigatioDlzGetNedPx(), navigatioDlzGetNedPy(), navigatioDlzGetNedPz());
        LOG_DEBUG(SYSTEM, "DLZ: biasX: %f, biasY: %f", biasX, biasY);
        LOG_DEBUG(SYSTEM, "DLZ: navpos z: %f", navGetCurrentActualPositionAndVelocity()->pos.z);
        LOG_DEBUG(SYSTEM, "DLZ: conf: %f, fade: %f", navigatioDlzGetConfidence(), dlzPosCtrlFade);
        LOG_DEBUG(SYSTEM, "DLZ: px: %f, py: %f", posErrorX, posErrorY);
    }

    // End Dlz here



*/