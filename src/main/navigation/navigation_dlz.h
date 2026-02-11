#pragma once

#include <stdio.h>
#include "navigation_private.h"
#include "drivers/time.h"

// C wrappers for C++ DLZ navigation class (implemented in navigation/dlz/nav_dlz_wrapper.cpp)
#ifdef __cplusplus
extern "C" {
#endif

void adum_dlz_init(void);

void adum_dlz_readskyvisdata(const uint8_t* bufferPtr, 
                             unsigned int dataSize);

void adum_dlz_reset(void);
                        
void adum_dlz_update(void);


const float adum_dlz_get_ned_pos_x(void);
const float adum_dlz_get_ned_pos_y(void);
const float adum_dlz_get_ned_vel_x(void);
const float adum_dlz_get_ned_vel_y(void);
const float adum_dlz_get_fade(void);

#ifdef __cplusplus
}
#endif

static inline float scaleRangeClippedf(float x, float srcMin, float srcMax, float destMin, float destMax) {
    float a = (destMax - destMin) * (x - srcMin);
    float b = srcMax - srcMin;
    float c = (a / b) + destMin;
    return fmin(fmax(c, destMin), destMax);
}