
#include "drivers/time.h"

#include "common/maths.h"
#include "common/vector.h"

#include "flight/imu.h"

#include "navigation/navigation_dlz.h"

#define UPDATE_TIMEOUT_MS 1000  // if no update in this time, DLZ is considered lost
#define MAX_DIST 1000.0f 
#define MAX_ALT 10000.0f // max 100 m
#define GAIN 2.0f

#define HOLD_ALT_CM 300.0f
#define HOLD_TIME_MS 3000


static volatile bool isNewDataReady = false;

static navDlzData_t navDlzData[2];
static volatile uint8_t activeBuffer = 0;

static float conditionedNavDlzPosX = 0.0f;
static float conditionedNavDlzPosY = 0.0f;
static float conditionedNavDlzPosZ = 0.0f;
static float conditionedNavDlzConfidence = 0.0f;

static float conditionedBiasPosX = 0.0f;
static float conditionedBiasPosY = 0.0f;

static bool holdOverDlzRequired = false;
static bool holdAllowed = true;
static uint32_t holdOverDlzStartTime = 0;

static navPosIntegrator_t navDlzPosIntegrator;

static navRateLimiter_t navDlzBiasPosXRateLimiter;
static navRateLimiter_t navDlzBiasPosYRateLimiter;
static navRateLimiter_t navDlzVspdRateLimiter;

const navDlzData_t * navigationDlzGetActiveBuffer(void);

static bool dlzDetectedFromHigh = false;

uint8_t skyvisFlag = 0;

// Skyvis status flag
//  UNDEFINED = 0
//  NO_RESPONSE = 10
//  COMMS_OK = 20
//  TAG_DETECTED = 30

void navigationDlzInit(void) {

    navDlzData[0].px = 0.0f;
    navDlzData[0].py = 0.0f;
    navDlzData[0].pz = 0.0f;
    navDlzData[0].confidence = 0.0f;
    navDlzData[0].timestamp_ms = 0;


    navDlzData[1].px = 0.0f;
    navDlzData[1].py = 0.0f;
    navDlzData[1].pz = 0.0f;
    navDlzData[1].confidence = 0.0f;
    navDlzData[1].timestamp_ms = 0;

    conditionedNavDlzPosX = 0.0f;
    conditionedNavDlzPosY = 0.0f;
    conditionedNavDlzPosZ = 0.0f;
    conditionedNavDlzConfidence = 0.0f;

    conditionedBiasPosX = 0.0f;
    conditionedBiasPosY = 0.0f;

    holdOverDlzRequired = false;
    holdAllowed = true;
    holdOverDlzStartTime = 0;

    // Integrator
    navPosIntegratorInit(&navDlzPosIntegrator, millis());

    // Rate limited 
    navRateLimiterInit(&navDlzBiasPosXRateLimiter, 50.0f, 0.0f, millis());
    navRateLimiterInit(&navDlzBiasPosYRateLimiter, 50.0f, 0.0f, millis());
    navRateLimiterInit(&navDlzVspdRateLimiter, 50.0f, 0.0f, millis());

    isNewDataReady = false;
    setSkyvisFlag(5);

}


void navigationDlzUpdate(const float posErrorX, const float posErrorY) {


    const navDlzData_t* data = navigationDlzGetActiveBuffer();

    if (millis() - data->timestamp_ms > UPDATE_TIMEOUT_MS) {

        conditionedNavDlzPosX = 0.0f;
        conditionedNavDlzPosY = 0.0f;
        conditionedNavDlzPosZ = 0.0f;
        conditionedNavDlzConfidence = 0.0f;

        conditionedBiasPosX = 0.0f;
        conditionedBiasPosY = 0.0f;

        isNewDataReady = false;
        navPosIntegratorReset(&navDlzPosIntegrator, millis());
        
        setSkyvisFlag(10); // No response
        return;
    }


    if (isNewDataReady) {

        fpVector3_t pos;

        pos.x = data->px;
        pos.y = data->py;
        pos.z = data->pz;
        // Below 200 cm altitude, confidence is only allowed to increase — prevents sudden
        // drops near the landing pad from destabilising bias estimation.
        // conditionedNavDlzPosZ here is the previous frame's value (Z updated later), which is fine.
        const float newConf = constrainf(data->confidence, 0.0f, 1.0f);
        const float altAbsCm = fabsf(conditionedNavDlzPosZ);
        if (altAbsCm > 10.0f && altAbsCm < HOLD_ALT_CM && dlzDetectedFromHigh) {
            conditionedNavDlzConfidence = fmaxf(newConf, conditionedNavDlzConfidence);
        } else {
            conditionedNavDlzConfidence = newConf;
        }

        isNewDataReady = false; // Mark data as consumed, so that we don't 
                                // use it again until new data arrives

        // Transform to body

        imuTransformVectorBodyToEarth(&pos);

        // Warning: Signs adjusted to match expected coordinate system of position error

        pos.x = -GAIN * pos.x;
        pos.y = -GAIN * pos.y;
        pos.z = pos.z;


        // Add integrator
        float ix, iy;
        navPosIntegratorUpdate(&navDlzPosIntegrator, 
                               pos.x, 
                               pos.y, 
                               millis(), 
                               &ix, &iy);
        
        if (conditionedNavDlzConfidence < 0.01f) {
            navPosIntegratorReset(&navDlzPosIntegrator, millis());
            ix = 0.0f;
            iy = 0.0f;
        }

        const float rawPosX = constrainf(pos.x + ix, -MAX_DIST, MAX_DIST);
        const float rawPosY = constrainf(pos.y + iy, -MAX_DIST, MAX_DIST);  

        // Calculate bias, rate limited
        conditionedBiasPosX = navRateLimiterUpdate(&navDlzBiasPosXRateLimiter,
                                                   conditionedNavDlzConfidence * (rawPosX - posErrorX),
                                                   millis());

        conditionedBiasPosY = navRateLimiterUpdate(&navDlzBiasPosYRateLimiter,
                                                   conditionedNavDlzConfidence * (rawPosY - posErrorY),
                                                   millis());
        
        conditionedNavDlzPosZ = constrainf(pos.z, -MAX_ALT, MAX_ALT);

        // For telemetry 

        if (conditionedNavDlzConfidence > 0.95f) {

            if (conditionedNavDlzPosZ > HOLD_ALT_CM) dlzDetectedFromHigh = true;

            setSkyvisFlag(30); // Tag detected
        } else {
            setSkyvisFlag(20); // Comms ok, but no tag detected
        }
    }

}




void navigationDlzReceiveNewData(const float px,
                                 const float py,
                                 const float pz,
                                 const float confidence) {

    uint8_t writeBuffer = 1 - activeBuffer;  // Write to inactive buffer
    
    navDlzData[writeBuffer].timestamp_ms = millis();
    navDlzData[writeBuffer].px = px;
    navDlzData[writeBuffer].py = py;
    navDlzData[writeBuffer].pz = pz;
    navDlzData[writeBuffer].confidence = confidence;
    
    activeBuffer = writeBuffer;  // Atomic pointer swap (8-bit write is atomic)
    isNewDataReady = true;
}



float navigationDlzUpdateAltCtrl(const bool landingInProgress, const float targetVel) {

    bool localRequireHold = false;
    const float altDlz = fabs(navigationDlzGetNedPz());

    // If close to landing, check position offset
    if (altDlz > 50.0f && altDlz < HOLD_ALT_CM && landingInProgress) {

        //if (conditionedNavDlzConfidence > 0.5f) {

            const float posErrMag = sqrtf(conditionedBiasPosX * conditionedBiasPosX + conditionedBiasPosY * conditionedBiasPosY);

            // Always required hold for now
            if (posErrMag > 200.0f) {
                localRequireHold = true;
            }
        //} 
    }

    if (localRequireHold && holdAllowed && !holdOverDlzRequired) {
        holdOverDlzRequired = true;
        holdOverDlzStartTime = millis();
    }


    if (holdOverDlzRequired && (millis() - holdOverDlzStartTime) > HOLD_TIME_MS) {
        holdOverDlzRequired = false;
        holdAllowed = false; // Don't allow hold again until reset, to prevent multiple holds in one flight
    }


    float targetVelOut = targetVel;


    if (holdOverDlzRequired) {
        targetVelOut = navRateLimiterUpdate(&navDlzVspdRateLimiter, 0.0f, millis());
    } else {
        targetVelOut = navRateLimiterUpdate(&navDlzVspdRateLimiter, targetVel, millis());
    }

    return targetVelOut;

}

void navigationDlzClearHoldBlocker(void) {
    holdAllowed = true;
    holdOverDlzRequired = false;
    dlzDetectedFromHigh = false;
}

void navigationDlzResetIntegrator(void) {
    navPosIntegratorReset(&navDlzPosIntegrator, millis());  
}


const navDlzData_t * navigationDlzGetActiveBuffer(void) {
    return &navDlzData[activeBuffer];
}


// Rate Limiter
void navRateLimiterInit(navRateLimiter_t *sl, 
                        const float rate_per_sec, 
                        const float initial_value, 
                        const uint32_t now_ms) {
    
    if(!sl) {
        //LOG_ERROR(SYSTEM, "navRateLimiterInit: null pointer");
        return;
    }

    sl->rate_per_sec = rate_per_sec;
    sl->last_output = initial_value;
    sl->last_time_ms = now_ms;
}


float navRateLimiterUpdate(navRateLimiter_t *sl, 
                           const float target, 
                           const uint32_t now_ms) {


    if(!sl) {
        //LOG_ERROR(SYSTEM, "navRateLimiterUpdate: null pointer");
        return 0.0f;
    }


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


uint32_t navigationDlzGetTimestamp(void) {
    return navigationDlzGetActiveBuffer()->timestamp_ms;
}

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

// Flags

void setSkyvisFlag(uint8_t flag) {
    skyvisFlag = flag;
}

uint8_t getSkyvisFlag(void) {
    return skyvisFlag;
}