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


void navigationDlzInit(void);

void navigationDlzUpdate(const float posErrorX, const float posErrorY);

void navigationDlzReceiveNewData(const float px,
                                 const float py,
                                 const float pz,
                                 const float confidence);

// Rate limiter

void navRateLimiterInit(navRateLimiter_t *sl, const float rate_per_sec, const float initial_value, const uint32_t now_ms);

float navRateLimiterUpdate(navRateLimiter_t *sl, const float target, const uint32_t now_ms);


// Getters

uint32_t navigationDlzGetTimestamp(void);

float navigationDlzGetBiasPosX(void);

float navigationDlzGetBiasPosY(void);

float navigationDlzGetNedPz(void);

float navigationDlzGetConfidence(void);