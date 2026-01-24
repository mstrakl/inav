#pragma once

#include <stdio.h>
#include "navigation_private.h"
#include "drivers/time.h"

typedef struct {
    bool        active; // If DLZ data is valid
    timeMs_t    lastUpdateTime; // millis
    //float       fadeValue;
    //bool        mspWpUpdate;

    int16_t     posX;         
    int16_t     posY;         
    int16_t     velZ;       // Vertical speed command in cm/s
    int16_t     gpsFade;    // How much gps signal to use (1000 - full cam, 0 - full gps)

                            
} NavDlzData_t;


typedef struct __attribute__((packed)) {
    int16_t  posX;
    int16_t  posY;
    int16_t  velZ;
    int16_t  gpsFade;
} mspSensorSkyvis_t;


extern NavDlzData_t NavDlzData;


// Func prototypes


void mspSkyvisReceiveNewData(
    const uint8_t * bufferPtr, 
    unsigned int dataSize);

// C wrappers for C++ DLZ navigation class (implemented in navigation/dlz/nav_dlz_wrapper.cpp)
#ifdef __cplusplus
extern "C" {
#endif

void adum_dlz_init(void);
void adum_dlz_update(timeMs_t currentTime);
void adum_dlz_reset(void);

#ifdef __cplusplus
}
#endif


static inline void navigationDLZReset(void) {
    NavDlzData.active = false;
    //NavDlzData.lastUpdateTime = 0; // Should not nullyfy !!!
    //NavDlzData.fadeValue = 0.0f;
    //NavDlzData.mspWpUpdate = false;
    NavDlzData.posX = 0;
    NavDlzData.posY = 0;
    NavDlzData.velZ = 0;
    NavDlzData.gpsFade = 0;
}


static inline float scaleRangeClippedf(float x, float srcMin, float srcMax, float destMin, float destMax) {
    float a = (destMax - destMin) * (x - srcMin);
    float b = srcMax - srcMin;
    float c = (a / b) + destMin;
    return fmin(fmax(c, destMin), destMax);
}