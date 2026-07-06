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

#pragma once

#define TARGET_BOARD_IDENTIFIER "AD41"
#define USBD_PRODUCT_STRING  "AIRDATAFC1"

#define USE_HARDWARE_PREBOOT_SETUP
#define USE_TARGET_CONFIG
#define USE_LOG
#define USE_BOOTLOG 2048

#define LED0                    PB5
#define LED1                    PB6

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3

#define USE_UART3
#define UART3_TX_PIN            NONE
#define UART3_RX_PIN            NONE

#define USE_UART4
#define UART4_TX_PIN            NONE
#define UART4_RX_PIN            PA1

#define USE_UART5
#define UART5_RX_PIN            PD2
#define UART5_TX_PIN            PC12

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define USE_VCP
#define SERIAL_PORT_COUNT       7  


// *************** SPI: SDCARD **********************

#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN        PA5
#define SPI1_MISO_PIN   	PA6
#define SPI1_MOSI_PIN   	PA7

// Enable SD card support over SPI
#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_BUS          BUS_SPI1
#define SDCARD_CS_PIN           PA4
#define USE_BLACKBOX


// *************** I2C /Baro/Mag/Pitot ********************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define DEFAULT_I2C_BUS         BUS_I2C1

#define USE_IMU_ICM42670
#define ICM42670_I2C_BUS        BUS_I2C1
#define IMU_ICM42670_ALIGN      CW0_DEG

#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP388


// *************** ADC *****************************


// ***************  OTHERS *************************

#define USE_GPS
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY)

//#define USE_DSHOT
//#define USE_ESC_SENSOR

#define TARGET_IO_PORTA         (0xFFFF & ~(BIT(13) | BIT(14)))  // Remove PA13.14 to free them for SWD
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2) | BIT(12) | BIT(13) | BIT(14) | BIT(15))
#define TARGET_IO_PORTE         (BIT(5) | BIT(6) | BIT(9) | BIT(11) | BIT(13) | BIT(14))

#define MAX_PWM_OUTPUT_PORTS      12
