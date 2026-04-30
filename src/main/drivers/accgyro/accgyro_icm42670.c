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
#include "drivers/accgyro/icm42670/inv_imu_defs.h"



#if defined(USE_IMU_ICM42670)


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

static void icm42670AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t * dev = gyro->busDev;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    /* Bring sensors out of reset and enable low-noise modes */
    busWrite(dev, (PWR_MGMT0 & 0xFF), PWR_MGMT0_ACCEL_MODE_LN | PWR_MGMT0_GYRO_MODE_LN);
    delay(15);

    /* Default: leave range/odr at device default (can be configured later) */
    /* Configure DRDY interrupt: pulsed, push-pull, active high */
    busWrite(dev, (INT_CONFIG & 0xFF), INT_CONFIG_INT1_MODE_PULSED | INT_CONFIG_INT1_DRIVE_CIRCUIT_PP | INT_CONFIG_INT1_POLARITY_HIGH);
    delay(15);

    /* Enable data ready interrupt source */
    busWrite(dev, (INT_SOURCE0 & 0xFF), 0x08);

    busSetSpeed(dev, BUS_SPEED_FAST);
}

static bool icm42670DeviceDetect(busDevice_t * dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    /* clear power management to allow WHO_AM_I read */
    busWrite(dev, (PWR_MGMT0 & 0xFF), 0x00);

    do {
        delay(150);

        busRead(dev, (WHO_AM_I & 0xFF), &tmp);

        if (tmp == INV_IMU_WHOAMI) {
            return true;
        }
        /* Retry detection */
    } while (attemptsRemaining--);

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
