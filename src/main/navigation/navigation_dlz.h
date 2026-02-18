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

void navigationDlzUpdate(const float posErrorX, const float posErrorY);

void navigationDlzReceiveNewData(const float px,
                                 const float py,
                                 const float pz,
                                 const float confidence);


// Getters

float navigationDlzGetBiasPosX(void);

float navigationDlzGetBiasPosY(void);

float navigationDlzGetNedPz(void);

float navigationDlzGetConfidence(void);