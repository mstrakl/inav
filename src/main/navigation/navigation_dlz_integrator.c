
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/maths.h"

#include "navigation/navigation_dlz.h"

#define POS_INT_KI            0.03f     // integrator gain
#define POS_INT_LIMIT         25.0f     // max action in cm

#define POS_INT_FULL_ERR      25.0f     // full integrator below this distance cm
#define POS_INT_ZERO_ERR      50.0f    // zero integrator above this distance cm

#define POS_INT_LEAK_ACTIVE   0.999f    // leak when active
#define POS_INT_RESET_ERR     100.0f

void navPosIntegratorInit(navPosIntegrator_t *pi, uint32_t now_ms) {
    if (!pi) return;

    pi->ix = 0.0f;
    pi->iy = 0.0f;
    pi->last_time_ms = now_ms;
}


static float navPosIntegratorWeight(float err)
{
    float aerr = fabsf(err);

    if (aerr <= POS_INT_FULL_ERR) {
        return 1.0f;
    }

    if (aerr >= POS_INT_ZERO_ERR) {
        return 0.0f;
    }

    // Linear fade between 50cm and 100cm
    return (POS_INT_ZERO_ERR - aerr) /
           (POS_INT_ZERO_ERR - POS_INT_FULL_ERR);
}


void navPosIntegratorUpdate(navPosIntegrator_t *pi,
                            const float errX,
                            const float errY,
                            const uint32_t now_ms,
                            float *outIx,
                            float *outIy)
{
    if (!pi || !outIx || !outIy) return;

    float dt = (now_ms - pi->last_time_ms) * 0.001f;
    pi->last_time_ms = now_ms;

    float bleedInactive = expf(-dt / 1.0f); 

    // Timing protection
    if (dt <= 0.001f || dt > 0.2f) {
        pi->ix *= bleedInactive;
        pi->iy *= bleedInactive;
        *outIx = pi->ix;
        *outIy = pi->iy;
        return;
    }

    // --- Compute weights ---
    float wx = navPosIntegratorWeight(errX);
    float wy = navPosIntegratorWeight(errY);

    // --- Integrate (scaled by weight) ---
    if (wx > 0.0f) {
        pi->ix += POS_INT_KI * errX * wx * dt;
        pi->ix *= POS_INT_LEAK_ACTIVE;
    } else {
        pi->ix *= bleedInactive;
    }

    if (wy > 0.0f) {
        pi->iy += POS_INT_KI * errY * wy * dt;
        pi->iy *= POS_INT_LEAK_ACTIVE;
    } else {
        pi->iy *= bleedInactive;
    }

    // --- Clamp ---
    pi->ix = constrainf(pi->ix, -POS_INT_LIMIT, POS_INT_LIMIT);
    pi->iy = constrainf(pi->iy, -POS_INT_LIMIT, POS_INT_LIMIT);

    // --- Safety reset ---
    if (fabsf(errX) > POS_INT_RESET_ERR) pi->ix = 0.0f;
    if (fabsf(errY) > POS_INT_RESET_ERR) pi->iy = 0.0f;

    *outIx = pi->ix;
    *outIy = pi->iy;
}


void navPosIntegratorReset(navPosIntegrator_t *pi, uint32_t now_ms) {
    if (!pi) return;

    pi->ix = 0.0f;
    pi->iy = 0.0f;
    pi->last_time_ms = now_ms;
}