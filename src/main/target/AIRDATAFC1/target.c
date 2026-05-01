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
#include <platform.h>

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"
#include "drivers/sensor.h"

#define ICM42670_I2C_ADDR (0x68)
BUSDEV_REGISTER_I2C(busdev_icm42670, DEVHW_ICM42670, ICM42670_I2C_BUS, ICM42670_I2C_ADDR, NONE, DEVFLAGS_NONE, IMU_ICM42670_ALIGN);

timerHardware_t timerHardware[] = {

    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_OUTPUT_AUTO,  0, 0),  // Output S1
    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_OUTPUT_AUTO,  0, 0),  // Output S2
    DEF_TIM(TIM4, CH1, PD12,  TIM_USE_OUTPUT_AUTO,  0, 0), // Output S3
    DEF_TIM(TIM4, CH2, PD13,  TIM_USE_OUTPUT_AUTO,  0, 0), // Output S4
    DEF_TIM(TIM4, CH3, PD14,  TIM_USE_OUTPUT_AUTO,  0, 0), // Output S5
    DEF_TIM(TIM4, CH4, PD15,  TIM_USE_OUTPUT_AUTO,  0, 0), // Output S6

    DEF_TIM(TIM9, CH1, PE5,  TIM_USE_OUTPUT_AUTO,  0, 0),   // Output S7
    DEF_TIM(TIM9, CH2, PE6,  TIM_USE_OUTPUT_AUTO,  0, 0),   // Output S8
    DEF_TIM(TIM1, CH1, PE9,  TIM_USE_OUTPUT_AUTO,  0, 0),   // Output S9
    DEF_TIM(TIM1, CH2, PE11,  TIM_USE_OUTPUT_AUTO,  0, 0),  // Output S10
    DEF_TIM(TIM1, CH3, PE13,  TIM_USE_OUTPUT_AUTO,  0, 0),  // Output S11
    DEF_TIM(TIM1, CH4, PE14,  TIM_USE_OUTPUT_AUTO,  0, 0),  // Output S12
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
