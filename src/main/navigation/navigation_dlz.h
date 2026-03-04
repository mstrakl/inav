#pragma once

#include <stdbool.h>
#include <stdint.h>


typedef struct {
    float px;
    float py;
    float pz;
    float confidence;
    uint32_t timestamp_ms;
} navDlzData_t;

typedef struct {
    float rate_per_sec;   // Maximum change per second
    float last_output;    // Previous output value
    uint32_t last_time_ms;
} navRateLimiter_t;

// Integrator
typedef struct {
    float ix;
    float iy;
    uint32_t last_time_ms;
} navPosIntegrator_t;


extern uint8_t skyvisFlag;

void navigationDlzInit(void);

void navigationDlzUpdate(const float posErrorX, const float posErrorY);

void navigationDlzReceiveNewData(const float px,
                                 const float py,
                                 const float pz,
                                 const float confidence);


float navigationDlzUpdateAltCtrl(const bool landingInProgress, const float targetVel);

void navigationDlzClearHoldBlocker(void);

void navigationDlzResetIntegrator(void);


// Integrator

void navPosIntegratorInit(navPosIntegrator_t *pi, uint32_t now_ms);

void navPosIntegratorUpdate(navPosIntegrator_t *pi,
                            const float errX,
                            const float errY,
                            const uint32_t now_ms,
                            float *outIx,
                            float *outIy);

void navPosIntegratorReset(navPosIntegrator_t *pi, uint32_t now_ms);

// Rate limiter

void navRateLimiterInit(navRateLimiter_t *sl, const float rate_per_sec, const float initial_value, const uint32_t now_ms);

float navRateLimiterUpdate(navRateLimiter_t *sl, const float target, const uint32_t now_ms);


// Getters

uint32_t navigationDlzGetTimestamp(void);

float navigationDlzGetBiasPosX(void);

float navigationDlzGetBiasPosY(void);

float navigationDlzGetNedPz(void);

float navigationDlzGetConfidence(void);

// Flags
void setSkyvisFlag(uint8_t flag);

uint8_t getSkyvisFlag(void);
