/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/log.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_icm42670.h"

#if defined(USE_IMU_ICM42670)

// Data registers
#define TEMP_DATA1        0x10009
#define ACCEL_DATA_X1     0x1000b
#define GYRO_DATA_X1      0x10011

#define PWR_MGMT0         0x1001f
#define PWR_MGMT0_GYRO_MODE_POS      0x02
#define PWR_MGMT0_GYRO_MODE_LN      (0x03 << PWR_MGMT0_GYRO_MODE_POS)
#define PWR_MGMT0_GYRO_MODE_LP      (0x02 << PWR_MGMT0_GYRO_MODE_POS)
#define PWR_MGMT0_GYRO_MODE_STANDBY (0x01 << PWR_MGMT0_GYRO_MODE_POS)
#define PWR_MGMT0_GYRO_MODE_OFF     (0x00 << PWR_MGMT0_GYRO_MODE_POS)
#define PWR_MGMT0_ACCEL_MODE_LN  0x03
#define PWR_MGMT0_ACCEL_MODE_LP  0x02
#define PWR_MGMT0_ACCEL_MODE_OFF 0x00
#define GYRO_CONFIG0      0x10020
#define ACCEL_CONFIG0     0x10021
#define GYRO_CONFIG1      0x10023
#define ACCEL_CONFIG1     0x10024
#define INTF_CONFIG0      0x10035

#define WHO_AM_I          0x10075
#define INTF_CONFIG0_SENSOR_DATA_ENDIAN_MASK (0x01 << 4)


// Configuration 

#define ICM42670_GYRO_UI_FS_2000DPS               (0x00 << 5)
#define ICM42670_ACCEL_UI_FS_16G                  (0x00 << 5)

#define ICM42670_ODR_1600HZ                       0x05
#define ICM42670_ODR_800HZ                        0x06
#define ICM42670_ODR_400HZ                        0x07
#define ICM42670_ODR_200HZ                        0x08
#define ICM42670_ODR_100HZ                        0x09
#define ICM42670_ODR_50HZ                         0x0A
#define ICM42670_ODR_25HZ                         0x0B
#define ICM42670_ODR_12HZ5                        0x0C

#define ICM42670_GYRO_UI_FILT_BW_BYPASS           0x00
#define ICM42670_GYRO_UI_FILT_BW_180HZ            0x01
#define ICM42670_GYRO_UI_FILT_BW_121HZ            0x02
#define ICM42670_GYRO_UI_FILT_BW_73HZ             0x03
#define ICM42670_GYRO_UI_FILT_BW_53HZ             0x04
#define ICM42670_GYRO_UI_FILT_BW_34HZ             0x05
#define ICM42670_GYRO_UI_FILT_BW_25HZ             0x06
#define ICM42670_GYRO_UI_FILT_BW_16HZ             0x07

#define ICM42670_ACCEL_UI_AVG_4X                  (0x01 << 4)

static const gyroFilterAndRateConfig_t icm42670GyroConfigs[] = {
    { GYRO_LPF_256HZ, 1600, { ICM42670_GYRO_UI_FS_2000DPS | ICM42670_ODR_1600HZ, ICM42670_ACCEL_UI_FS_16G | ICM42670_ODR_1600HZ } },
    { GYRO_LPF_256HZ, 800,  { ICM42670_GYRO_UI_FS_2000DPS | ICM42670_ODR_800HZ,  ICM42670_ACCEL_UI_FS_16G | ICM42670_ODR_800HZ } },
    { GYRO_LPF_256HZ, 400,  { ICM42670_GYRO_UI_FS_2000DPS | ICM42670_ODR_400HZ,  ICM42670_ACCEL_UI_FS_16G | ICM42670_ODR_400HZ } },
    { GYRO_LPF_256HZ, 200,  { ICM42670_GYRO_UI_FS_2000DPS | ICM42670_ODR_200HZ,  ICM42670_ACCEL_UI_FS_16G | ICM42670_ODR_200HZ } },
    { GYRO_LPF_256HZ, 100,  { ICM42670_GYRO_UI_FS_2000DPS | ICM42670_ODR_100HZ,  ICM42670_ACCEL_UI_FS_16G | ICM42670_ODR_100HZ } },
    { GYRO_LPF_256HZ, 50,   { ICM42670_GYRO_UI_FS_2000DPS | ICM42670_ODR_50HZ,   ICM42670_ACCEL_UI_FS_16G | ICM42670_ODR_50HZ } },
    { GYRO_LPF_256HZ, 25,   { ICM42670_GYRO_UI_FS_2000DPS | ICM42670_ODR_25HZ,   ICM42670_ACCEL_UI_FS_16G | ICM42670_ODR_25HZ } },
    { GYRO_LPF_256HZ, 12,   { ICM42670_GYRO_UI_FS_2000DPS | ICM42670_ODR_12HZ5,  ICM42670_ACCEL_UI_FS_16G | ICM42670_ODR_12HZ5 } },
};

static uint8_t icm42670FilterBwFromLpf(uint8_t lpf)
{
    switch (lpf) {
    case GYRO_LPF_NONE:
        return ICM42670_GYRO_UI_FILT_BW_BYPASS;
    case GYRO_LPF_256HZ:
    case GYRO_LPF_188HZ:
        return ICM42670_GYRO_UI_FILT_BW_180HZ;
    case GYRO_LPF_98HZ:
        return ICM42670_GYRO_UI_FILT_BW_121HZ;
    case GYRO_LPF_42HZ:
        return ICM42670_GYRO_UI_FILT_BW_34HZ;
    case GYRO_LPF_20HZ:
        return ICM42670_GYRO_UI_FILT_BW_16HZ;
    case GYRO_LPF_10HZ:
    case GYRO_LPF_5HZ:
        return ICM42670_GYRO_UI_FILT_BW_16HZ;
    }

    return ICM42670_GYRO_UI_FILT_BW_BYPASS;
}

static void formatByteBinary(char *buf, uint8_t value)
{
    buf[0] = '0';
    buf[1] = 'b';

    for (int bit = 0; bit < 8; bit++) {
        buf[2 + bit] = (value & (1 << (7 - bit))) ? '1' : '0';
    }

    buf[10] = '\0';
}

static void icm42670AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 4;
}

static bool icm42670AccRead(accDev_t *acc)
{
    uint8_t data[6];

    const bool ack = busReadBuf(acc->busDev, (ACCEL_DATA_X1 & 0xFF), data, 6);
    if (!ack) {
        return false;
    }

    acc->ADCRaw[X] = (float) int16_val_big_endian(data, 0);
    acc->ADCRaw[Y] = (float) int16_val_big_endian(data, 1);
    acc->ADCRaw[Z] = (float) int16_val_big_endian(data, 2);

    return true;
}

bool icm42670AccDetect(accDev_t *acc)
{
    acc->busDev = busDeviceOpen(BUSTYPE_ANY, DEVHW_ICM42670, acc->imuSensorToUse);
    
    if (acc->busDev == NULL) {
        return false;
    }

    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(acc->busDev);
    if (ctx->chipMagicNumber != 0x4267) {
        return false;
    }

    acc->initFn = icm42670AccInit;
    acc->readFn = icm42670AccRead;
    acc->accAlign = acc->busDev->param;

    return true;
}

static void icm42670AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t * dev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = chooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs,
                                                                &icm42670GyroConfigs[0], ARRAYLEN(icm42670GyroConfigs));

    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    /* Bring sensors out of reset and enable low-noise modes */
    busWrite(dev, (PWR_MGMT0 & 0xFF), PWR_MGMT0_ACCEL_MODE_LN | PWR_MGMT0_GYRO_MODE_LN);
    delay(15);

    /* ODR and dynamic range */
    busWrite(dev, (GYRO_CONFIG0 & 0xFF), config->gyroConfigValues[0]);
    delay(15);

    busWrite(dev, (ACCEL_CONFIG0 & 0xFF), config->gyroConfigValues[1]);
    delay(15);

    /* Low latency filter bandwidth */
    busWrite(dev, (GYRO_CONFIG1 & 0xFF), icm42670FilterBwFromLpf(gyro->lpf));
    delay(15);
    
    // Also add averagning 4x for accel
    busWrite(dev, (ACCEL_CONFIG1 & 0xFF), icm42670FilterBwFromLpf(gyro->lpf) | ICM42670_ACCEL_UI_AVG_4X);
    delay(15);

    /* Sensor data and FIFO readout use the same byte order as the driver */
    busWrite(dev, (INTF_CONFIG0 & 0xFF), INTF_CONFIG0_SENSOR_DATA_ENDIAN_MASK);
    delay(15);

    busSetSpeed(dev, BUS_SPEED_FAST);

    uint8_t c1;
    busRead(dev, (GYRO_CONFIG0 & 0xFF), &c1);
    delay(15);

    uint8_t c2;
    busRead(dev, (ACCEL_CONFIG0 & 0xFF), &c2);
    delay(15);

    uint8_t c3;
    busRead(dev, (GYRO_CONFIG1 & 0xFF), &c3);
    delay(15);

    uint8_t c4;
    busRead(dev, (ACCEL_CONFIG1 & 0xFF), &c4);
    delay(15);

    LOG_INFO(GYRO, "Gyro ICM42670 configured with:");
    LOG_INFO(GYRO, "  ODR: %d Hz, sample interval: %ld us", config->gyroRateHz, gyro->sampleRateIntervalUs);

    {
        char gyroCfg0Binary[11];
        char accelCfg0Binary[11];
        char gyroCfg1Binary[11];
        char accelCfg1Binary[11];

        formatByteBinary(gyroCfg0Binary, c1);
        formatByteBinary(accelCfg0Binary, c2);
        formatByteBinary(gyroCfg1Binary, c3);
        formatByteBinary(accelCfg1Binary, c4);

        LOG_INFO(GYRO, "  GyroCfg0_Byte: 0x%02X (%s)", c1, gyroCfg0Binary);
        LOG_INFO(GYRO, "  AccelCfg0_Byte: 0x%02X (%s)", c2, accelCfg0Binary);
        LOG_INFO(GYRO, "  GyroCfg1_Byte: 0x%02X (%s)", c3, gyroCfg1Binary);
        LOG_INFO(GYRO, "  AccelCfg1_Byte: 0x%02X (%s)", c4, accelCfg1Binary);
    }

    gyro->scale = 1.0f / 16.4f;

}

static bool icm42670DeviceDetect(busDevice_t * dev)
{
    uint8_t tmp = 0xFF;
    uint8_t attemptsRemaining = 5;
    volatile bool status = false;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    /* clear power management to allow WHO_AM_I read */
    busWrite(dev, (PWR_MGMT0 & 0xFF), 0x00);

    do {
        delay(150);

        status = busRead(dev, (WHO_AM_I & 0xFF), &tmp);
        
        // Return of icm42670 who_am_i is 0x67
        if (tmp == 0x67) {
            return true;
        }
        /* Retry detection */
    } while (attemptsRemaining--);

    if (!status && tmp == 0xFF) {
        tmp = 0xFE;
    }

    return false;
}

static bool icm42670GyroRead(gyroDev_t *gyro)
{
    uint8_t data[6];

    const bool ack = busReadBuf(gyro->busDev, (GYRO_DATA_X1 & 0xFF), data, 6);
    if (!ack) {
        return false;
    }

    gyro->gyroADCRaw[X] = (float) int16_val_big_endian(data, 0);
    gyro->gyroADCRaw[Y] = (float) int16_val_big_endian(data, 1);
    gyro->gyroADCRaw[Z] = (float) int16_val_big_endian(data, 2);

    return true;
}

static bool icm42670ReadTemperature(gyroDev_t *gyro, int16_t * temp)
{
    uint8_t data[2];

    const bool ack = busReadBuf(gyro->busDev, (TEMP_DATA1 & 0xFF), data, 2);
    if (!ack) {
        return false;
    }
    /* Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25 */
    *temp = ( int16_val_big_endian(data, 0) / 13.248 ) + 250; /* degC * 10 */

    return true;
}

bool icm42670GyroDetect(gyroDev_t *gyro)
{
    gyro->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_ICM42670, gyro->imuSensorToUse, OWNER_MPU);
    if (gyro->busDev == NULL) {
        return false;
    }

    if (!icm42670DeviceDetect(gyro->busDev)) {
        busDeviceDeInit(gyro->busDev);
        return false;
    }

    /* Magic number for ACC detection to indicate that we have detected icm42670 gyro */
    mpuContextData_t * ctx = busDeviceGetScratchpadMemory(gyro->busDev);
    ctx->chipMagicNumber = 0x4267;

    gyro->initFn = icm42670AccAndGyroInit;
    gyro->readFn = icm42670GyroRead;
    gyro->intStatusFn = gyroCheckDataReady;
    gyro->temperatureFn = icm42670ReadTemperature;
    gyro->scale = 1.0f / 16.4f;     /* default scale; can be adjusted per FSR */
    gyro->gyroAlign = gyro->busDev->param;

    return true;
}


#endif

/*


#ifdef USE_IMU_FAKE

static float fakeGyroADC[XYZ_AXIS_COUNT];

static void fakeGyroInit(gyroDev_t *gyro)
{
    UNUSED(gyro);
}

void fakeGyroSet(int16_t x, int16_t y, int16_t z)
{
    fakeGyroADC[X] = x;
    fakeGyroADC[Y] = y;
    fakeGyroADC[Z] = z;
}

static bool fakeGyroRead(gyroDev_t *gyro)
{
    gyro->gyroADCRaw[X] = fakeGyroADC[X];
    gyro->gyroADCRaw[Y] = fakeGyroADC[Y];
    gyro->gyroADCRaw[Z] = fakeGyroADC[Z];
    return true;
}

static bool fakeGyroReadTemperature(gyroDev_t *gyro, int16_t *temperatureData)
{
    UNUSED(gyro);
    UNUSED(temperatureData);
    return true;
}

static bool fakeGyroInitStatus(gyroDev_t *gyro)
{
    UNUSED(gyro);
    return true;
}

bool fakeGyroDetect(gyroDev_t *gyro)
{
    gyro->initFn = fakeGyroInit;
    gyro->intStatusFn = fakeGyroInitStatus;
    gyro->readFn = fakeGyroRead;
    gyro->temperatureFn = fakeGyroReadTemperature;
    gyro->scale = 0.0625f;
    gyro->gyroAlign = 0;
    return true;
}

static int16_t fakeAccData[XYZ_AXIS_COUNT];

static void fakeAccInit(accDev_t *acc)
{
    acc->acc_1G = 9806;
}

void fakeAccSet(int16_t x, int16_t y, int16_t z)
{ 
    fakeAccData[X] = x;
    fakeAccData[Y] = y;
    fakeAccData[Z] = z;
}

static bool fakeAccRead(accDev_t *acc)
{
    acc->ADCRaw[X] = fakeAccData[X];
    acc->ADCRaw[Y] = fakeAccData[Y];
    acc->ADCRaw[Z] = fakeAccData[Z];
    return true;
}

bool fakeAccDetect(accDev_t *acc)
{
    acc->initFn = fakeAccInit;
    acc->readFn = fakeAccRead;
    acc->accAlign = 0;
    return true;
}
*/
