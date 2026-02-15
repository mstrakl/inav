#pragma once

#include <stdbool.h>


typedef struct {
    int32_t nedPx;
    int32_t nedPy;
    int16_t confidence;
} navDlzData_t;


void navigationDlzInit(void);

void navigationDlzUpdate(void);

void navigationDlzReceiveNewData(
    uint8_t *bufferPtr, 
    unsigned int dataSize);



const float navigatioDlzGetNedPx(void);

const float navigatioDlzGetNedPy(void);

const float navigatioDlzGetConfidence(void);
