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


void navigationDlzInit(void);

void navigationDlzUpdate(float posErrorX, float posErrorY);

void navigationDlzReceiveNewData(const float px,
                                 const float py,
                                 const float pz,
                                 const float confidence);


// Getters

float navigatioDlzGetNedPx(void);

float navigatioDlzGetNedPy(void);

float navigatioDlzGetNedPz(void);

float navigatioDlzGetConfidence(void);