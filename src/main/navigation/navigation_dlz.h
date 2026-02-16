#pragma once

#include <stdbool.h>
#include <stdint.h>


typedef struct {
    int32_t nedPx;
    int32_t nedPy;
    int32_t nedPz;
    int16_t confidence;
} navDlzData_t;


void navigationDlzInit(void);

void navigationDlzUpdate(void);

void navigationDlzReceiveNewData(
    uint8_t *bufferPtr,
    unsigned int dataSize);

/* Internal helper is static in the C file and should not be declared here. */

float navigatioDlzGetNedPx(void);

float navigatioDlzGetNedPy(void);

float navigatioDlzGetNedPz(void);

float navigatioDlzGetConfidence(void);