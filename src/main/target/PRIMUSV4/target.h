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

#ifdef __cplusplus
 extern "C" {
#endif

#define TARGET_BOARD_IDENTIFIER "PRIMUS_V3R" // CJMCU
#define USE_HARDWARE_REVISION_DETECTION

#define LED0_GPIO GPIOC
#define LED0_PIN Pin_14 // PC14 (LED)
#define LED_M
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOC

#define LED1_GPIO GPIOC
#define LED1_PIN Pin_13 // PC13 (LED)
#define LED_R
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOC


#define LED2_GPIO GPIOC
#define LED2_PIN Pin_15 // PC15 (LED)
#define LED_L
#define LED2_PERIPHERAL RCC_APB2Periph_GPIOC

#define LED3_GPIO GPIOB
#define LED3_PIN Pin_5 // PA0 (LED)
#define LED3
#define LED3_PERIPHERAL RCC_APB2Periph_GPIOB

#define LED4_GPIO GPIOB
#define LED4_PIN Pin_4 // PA0 (LED)
#define LED4
#define LED4_PERIPHERAL RCC_APB2Periph_GPIOB
//#define LED3_INVERTED

#define ACC
#define USE_ACC_MPU6500

#define GYRO
#define USE_GYRO_MPU6500

#define MAG
#define USE_MAG_AK8963
#define MAG_AK8963_ALIGN CW90_DEG_FLIP
//#define MAG_ENFORCE //DA: To ensure mag calibration before start. Disable for faster dev options.

#define BARO
#define USE_BARO_MS5611

//#define LASER_TOF

#define BRUSHED_MOTORS

#define USE_USART1
#define USE_USART2

#define SERIAL_PORT_COUNT 2

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)


//#define USE_FLASHFS
//#define USE_FLASH_M25P16

// #define SOFT_I2C // enable to test software i2c
// #define SOFT_I2C_PB1011 // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

#define SERIAL_RX
//#define USE_SERVOS
//#define USE_CLI

#define SPEKTRUM_BIND
#define BIND_PORT  GPIOA
#define BIND_PIN   Pin_3

// Since the CJMCU PCB has holes for 4 motors in each corner we can save same flash space by disabling support for other mixers.
#define USE_QUAD_MIXER_ONLY


#if (FLASH_SIZE > 64)
//#define BLACKBOX
#endif

//#undef USE_CLI
//#define GTUNE
//#define BLACKBOX
#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_3
#define VBAT_ADC_CHANNEL            ADC_Channel_3

#define FLIGHT_STATUS_INDICATOR
#define ENABLE_ACROBAT
#define LED_ENABLE
#define TEST_ENABLE
#define LOGIC_3_V

#ifdef __cplusplus
 }
#endif
