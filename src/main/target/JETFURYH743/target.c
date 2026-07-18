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

#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"
#include "drivers/sensor.h"


#define ICM42670_I2C_ADDR (0x68)
BUSDEV_REGISTER_I2C(busdev_icm42670, DEVHW_ICM42670, ICM42670_I2C_BUS, ICM42670_I2C_ADDR, NONE, DEVFLAGS_NONE, IMU_ICM42670_ALIGN);

timerHardware_t timerHardware[] = {
    DEF_TIM(TIM4,  CH1, PD12,  TIM_USE_OUTPUT_AUTO, 0, 0), // S1  DMA1_ST0
    DEF_TIM(TIM4,  CH2, PD13,  TIM_USE_OUTPUT_AUTO, 0, 1), // S2  DMA1_ST1
    DEF_TIM(TIM4,  CH3, PD14,  TIM_USE_OUTPUT_AUTO, 0, 2), // S3  DMA1_ST2
    DEF_TIM(TIM4,  CH4, PD15,  TIM_USE_OUTPUT_AUTO, 0, 3), // S4  DMA1_ST3
    DEF_TIM(TIM8,  CH3, PC8,   TIM_USE_OUTPUT_AUTO, 0, 4), // S5  DMA1_ST4
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
