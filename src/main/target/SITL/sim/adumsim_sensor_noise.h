/*
 * Simple sensor noise/bias/lag simulator for ADUM SITL
 * Intended to be included only by SITL sim code (C usage)
 *
 * Usage:
 *  - call `adumSimNoiseInit()` once at sim startup
 *  - call `adumSimApplySensorNoise(...)` periodically from the sim update/listen worker
 *
 * The file implements Gaussian noise, slow bias random walk and a first-order
 * low-pass to emulate lag for the following signals:
 *  - accel x,y,z  (m/s^2)
 *  - gyro x,y,z   (deg/s)
 *  - roll, pitch   (deg)
 *  - mag north x,y,z (arbitrary units coming from sim)
 *  - gps lat/lon (1e7 degrees int), groundspeed (m/s), hpath/track (deg)
 *
 * The implementation is self-contained and tunable via the DEFAULT_* macros below.
 */

#ifndef ADUMSIM_SENSOR_NOISE_H
#define ADUMSIM_SENSOR_NOISE_H

#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

// Default tuning parameters (can be changed by editing this header or
// by providing setter functions in future)
#define DEFAULT_ACCEL_NOISE_STD_MS2      0.02f   // m/s^2
#define DEFAULT_ACCEL_BIAS_WALK_STD_MS2  0.002f  // m/s^2 / sqrt(s)
#define DEFAULT_ACCEL_LAG_TAU_S          0.02f   // seconds

#define DEFAULT_GYRO_NOISE_STD_DPS       0.02f   // deg/s
#define DEFAULT_GYRO_BIAS_WALK_STD_DPS   0.001f  // deg/s / sqrt(s)
#define DEFAULT_GYRO_LAG_TAU_S           0.005f  // seconds

#define DEFAULT_ANGLE_NOISE_STD_DEG      0.05f   // deg
#define DEFAULT_ANGLE_BIAS_WALK_STD_DEG  0.002f  // deg / sqrt(s)
#define DEFAULT_ANGLE_LAG_TAU_S          0.02f   // seconds

#define DEFAULT_MAG_NOISE_STD            0.1f
#define DEFAULT_MAG_BIAS_WALK_STD        0.01f
#define DEFAULT_MAG_LAG_TAU_S            0.05f

#define DEFAULT_GPS_POS_NOISE_M          2.0f    // meters
#define DEFAULT_GPS_POS_BIAS_WALK_M      0.1f    // meters / sqrt(s)
#define DEFAULT_GPS_POS_LAG_TAU_S        1.0f    // seconds

#define DEFAULT_GS_NOISE_MPS             0.1f    // m/s
#define DEFAULT_GS_BIAS_WALK_MPS         0.01f   // m/s / sqrt(s)
#define DEFAULT_GS_LAG_TAU_S             0.5f

#define DEFAULT_HPATH_NOISE_DEG          1.0f    // deg
#define DEFAULT_HPATH_BIAS_WALK_DEG      0.1f    // deg / sqrt(s)
#define DEFAULT_HPATH_LAG_TAU_S          0.5f

// Helper: simple Gaussian generator using Box-Muller transform
static inline float adumSimGaussianNoise(void)
{
    static int haveSpare = 0;
    static float spare;

    if (haveSpare) {
        haveSpare = 0;
        return spare;
    }

    float u, v, s;
    do {
        u = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
        v = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
        s = u*u + v*v;
    } while (s == 0.0f || s >= 1.0f);

    float mul = sqrtf(-2.0f * logf(s) / s);
    spare = v * mul;
    haveSpare = 1;
    return u * mul;
}

// Convert meters to 1e7-degrees for latitude
static inline int32_t adumSimMetersTo1e7Lat(float meters)
{
    const float meters_per_deg_lat = 111319.5f; // approximate
    return (int32_t)lrintf(meters * (10000000.0f / meters_per_deg_lat));
}

// Convert meters east to 1e7-degrees longitude at given latitude
static inline int32_t adumSimMetersTo1e7Lon(float meters_east, int32_t lat_1e7)
{
    const float meters_per_deg_lat = 111319.5f;
    float lat_deg = lat_1e7 / 1e7f;
    float scale = cosf(lat_deg * (float)M_PI / 180.0f);
    if (scale < 0.0001f) scale = 0.0001f;
    return (int32_t)lrintf(meters_east * (10000000.0f / (meters_per_deg_lat * scale)));
}

// Internal state for a 3-axis signal
typedef struct {
    float bias[3];        // slowly varying bias (random walk)
    float lagState[3];    // first-order lag output
} adumSimVecState_t;

// Internal state for scalar signals
typedef struct {
    float bias;
    float lagState;
} adumSimScalarState_t;

// Global static state (file-local via header static variables)
static adumSimVecState_t s_accelState = {{0,0,0},{0,0,0}};
static adumSimVecState_t s_gyroState = {{0,0,0},{0,0,0}};
static adumSimVecState_t s_magState = {{0,0,0},{0,0,0}};
static adumSimScalarState_t s_rollState = {0,0};
static adumSimScalarState_t s_pitchState = {0,0};
static adumSimScalarState_t s_gsState = {0,0};
static adumSimScalarState_t s_hpathState = {0,0};
static int s_seeded = 0;

// Initialize RNG and optional initial bias
static inline void adumSimNoiseInit(void)
{
    if (!s_seeded) {
        srand((unsigned int)time(NULL));
        s_seeded = 1;
    }
    // zero states by default (already zeroed statically)
}

// Core helper: apply gaussian noise + bias random walk + first-order lag
static inline float adumSimApplySignalNoise(float raw, float noiseStd, float biasWalkStd, float tau, float *biasPtr, float *lagPtr, float dt)
{
    // bias random walk
    if (biasWalkStd > 0.0f) {
        // scale random walk by sqrt(dt) to make it independent of update rate
        float step = adumSimGaussianNoise() * biasWalkStd * sqrtf(dt);
        *biasPtr += step;
    }

    float noise = (noiseStd > 0.0f) ? adumSimGaussianNoise() * noiseStd : 0.0f;
    float noisy = raw + *biasPtr + noise;

    // first order lag: y += (x - y) * (dt / (tau + dt))
    if (tau <= 0.0f) {
        *lagPtr = noisy;
    } else {
        float alpha = dt / (tau + dt);
        *lagPtr += (noisy - *lagPtr) * alpha;
    }
    return *lagPtr;
}

// Main API: apply noise to the listed sensors.
// - lat_1e7 and lon_1e7 are modified in-place (they are integer 1e7-degree values coming from the sim)
// - dt is the elapsed time in seconds since last call (pass small positive value, e.g. 0.01f)
static inline void adumSimApplySensorNoise(
    float *accel_x, float *accel_y, float *accel_z,
    float *gyro_x, float *gyro_y, float *gyro_z,
    float *roll, float *pitch,
    float *mag_x, float *mag_y, float *mag_z,
    int32_t *lat_1e7, int32_t *lon_1e7,
    float *groundspeed, float *hpath,
    float dt)
{
    if (dt <= 0.0f) dt = 0.02f; // default if caller forgets

    // accel
    float a[3] = {*accel_x, *accel_y, *accel_z};
    for (int i=0;i<3;i++) {
        a[i] = adumSimApplySignalNoise(a[i], DEFAULT_ACCEL_NOISE_STD_MS2, DEFAULT_ACCEL_BIAS_WALK_STD_MS2, DEFAULT_ACCEL_LAG_TAU_S, &s_accelState.bias[i], &s_accelState.lagState[i], dt);
    }
    *accel_x = a[0]; *accel_y = a[1]; *accel_z = a[2];

    // gyro
    float g[3] = {*gyro_x, *gyro_y, *gyro_z};
    for (int i=0;i<3;i++) {
        g[i] = adumSimApplySignalNoise(g[i], DEFAULT_GYRO_NOISE_STD_DPS, DEFAULT_GYRO_BIAS_WALK_STD_DPS, DEFAULT_GYRO_LAG_TAU_S, &s_gyroState.bias[i], &s_gyroState.lagState[i], dt);
    }
    *gyro_x = g[0]; *gyro_y = g[1]; *gyro_z = g[2];

    // roll / pitch (angles)
    *roll = adumSimApplySignalNoise(*roll, DEFAULT_ANGLE_NOISE_STD_DEG, DEFAULT_ANGLE_BIAS_WALK_STD_DEG, DEFAULT_ANGLE_LAG_TAU_S, &s_rollState.bias, &s_rollState.lagState, dt);
    *pitch = adumSimApplySignalNoise(*pitch, DEFAULT_ANGLE_NOISE_STD_DEG, DEFAULT_ANGLE_BIAS_WALK_STD_DEG, DEFAULT_ANGLE_LAG_TAU_S, &s_pitchState.bias, &s_pitchState.lagState, dt);

    // magnetometer (north vector sample)
    float m[3] = {*mag_x, *mag_y, *mag_z};
    for (int i=0;i<3;i++) {
        m[i] = adumSimApplySignalNoise(m[i], DEFAULT_MAG_NOISE_STD, DEFAULT_MAG_BIAS_WALK_STD, DEFAULT_MAG_LAG_TAU_S, &s_magState.bias[i], &s_magState.lagState[i], dt);
    }
    *mag_x = m[0]; *mag_y = m[1]; *mag_z = m[2];

    // GPS position: treat lat/lon noise as small east/north offsets in meters
    // add independent north/east noise then convert to 1e7-degree deltas
    float northNoiseM = adumSimGaussianNoise() * DEFAULT_GPS_POS_NOISE_M;
    float eastNoiseM  = adumSimGaussianNoise() * DEFAULT_GPS_POS_NOISE_M;

    // bias random walk for position (approx meters)
    static float s_gpsBiasNorth = 0.0f, s_gpsBiasEast = 0.0f;
    if (DEFAULT_GPS_POS_BIAS_WALK_M > 0.0f) {
        s_gpsBiasNorth += adumSimGaussianNoise() * DEFAULT_GPS_POS_BIAS_WALK_M * sqrtf(dt);
        s_gpsBiasEast  += adumSimGaussianNoise() * DEFAULT_GPS_POS_BIAS_WALK_M * sqrtf(dt);
    }

    float northTotal = northNoiseM + s_gpsBiasNorth;
    float eastTotal  = eastNoiseM + s_gpsBiasEast;

    // Apply simple lag to position (scalar) using s_gsState.lagState as temporary storage for meters
    static float s_gpsLagNorth = 0.0f, s_gpsLagEast = 0.0f;
    if (DEFAULT_GPS_POS_LAG_TAU_S > 0.0f) {
        float alpha = dt / (DEFAULT_GPS_POS_LAG_TAU_S + dt);
        s_gpsLagNorth += (northTotal - s_gpsLagNorth) * alpha;
        s_gpsLagEast  += (eastTotal - s_gpsLagEast) * alpha;
    } else {
        s_gpsLagNorth = northTotal;
        s_gpsLagEast = eastTotal;
    }

    // convert meters to 1e7 degrees and add to lat/lon
    if (lat_1e7 && lon_1e7) {
        int32_t dlat = adumSimMetersTo1e7Lat(s_gpsLagNorth);
        int32_t dlon = adumSimMetersTo1e7Lon(s_gpsLagEast, *lat_1e7);
        *lat_1e7 = *lat_1e7 + dlat;
        *lon_1e7 = *lon_1e7 + dlon;
    }

    // groundspeed (m/s)
    *groundspeed = adumSimApplySignalNoise(*groundspeed, DEFAULT_GS_NOISE_MPS, DEFAULT_GS_BIAS_WALK_MPS, DEFAULT_GS_LAG_TAU_S, &s_gsState.bias, &s_gsState.lagState, dt);

    // hpath/track (degrees)
    *hpath = adumSimApplySignalNoise(*hpath, DEFAULT_HPATH_NOISE_DEG, DEFAULT_HPATH_BIAS_WALK_DEG, DEFAULT_HPATH_LAG_TAU_S, &s_hpathState.bias, &s_hpathState.lagState, dt);
}

#endif // ADUMSIM_SENSOR_NOISE_H
