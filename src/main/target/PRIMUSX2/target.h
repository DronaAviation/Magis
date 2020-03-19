/*
 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#ifdef __cplusplus
extern "C" {
#endif 
#define TARGET_BOARD_IDENTIFIER "PRIMUSX2"

/* #define LED0_GPIO   GPIOB
 #define LED0_PIN    Pin_4 // Blue LEDs - PB4
 #define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB
 #define LED1_GPIO   GPIOB
 #define LED1_PIN    Pin_5  // Green LEDs - PB5
 #define LED1_PERIPHERAL RCC_AHBPeriph_GPIOB */
//#define BEEP_GPIO   GPIOA
//#define BEEP_PIN    Pin_5  // White LEDs - PA5
//#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOA
#define LED0_GPIO GPIOC
#define LED0_PIN Pin_14 // PC14 (LED)	//Blue
#define LED_R
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOC

#define LED1_GPIO GPIOC
#define LED1_PIN Pin_13 // PC13 (LED)	//Red
#define LED_L
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOC

#define LED2_GPIO GPIOC
#define LED2_PIN Pin_15 // PC15 (LED)	//Green
#define LED_M
#define LED2_PERIPHERAL RCC_AHBPeriph_GPIOC

#define USABLE_TIMER_CHANNEL_COUNT 5


#define M25P16_CS_GPIO        GPIOB
#define M25P16_CS_PIN         GPIO_Pin_12
#define M25P16_SPI_INSTANCE   SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16



#define ACC
#define USE_ACC_ICM20948
//#define ACC_ICM20948_ALIGN CW90_DEG_FLIP	//For plutoX-new V1 comment else for old uncomment

#define GYRO
#define USE_GYRO_ICM20948
//#define GYRO_ICM20948_ALIGN CW90_DEG_FLIP //For plutoX-new comment  else for old uncomment

#define MAG
#define USE_MAG_AK09916
#define MAG_AK09916_ALIGN CW180_DEG_FLIP //For plutoX-new remove comment  else for old comment
//#define MAG_ENFORCE

#define BARO
#define USE_BARO_ICP10111

#define BRUSHED_MOTORS

//#define BEEPER
#define LED0
#define LED1

//#define USE_VCP
#define USE_USART1 	//Connected to ESP module
#define USE_USART2 	//Unibus
//#define USE_USART3 //Unibus, but cannot be used if Motor 7 and 8 are being used

#define SERIAL_PORT_COUNT 4

#define UART1_TX_PIN        GPIO_Pin_9 // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_2 // PA2
#define UART2_RX_PIN        GPIO_Pin_3 // PA3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource2
#define UART2_RX_PINSOURCE  GPIO_PinSource3

/* #define UART3_TX_PIN        GPIO_Pin_10 // PB10 (AF7)
 #define UART3_RX_PIN        GPIO_Pin_11 // PB11 (AF7)
 #define UART3_GPIO_AF       GPIO_AF_7
 #define UART3_GPIO          GPIOB
 #define UART3_TX_PINSOURCE  GPIO_PinSource10
 #define UART3_RX_PINSOURCE  GPIO_PinSource11 */

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1) // SDA (PB9/AF4), SCL (PB8/AF4)

#define I2C1_SCL_GPIO        GPIOB
#define I2C1_SCL_GPIO_AF     GPIO_AF_4
#define I2C1_SCL_PIN         GPIO_Pin_8
#define I2C1_SCL_PIN_SOURCE  GPIO_PinSource8
#define I2C1_SCL_CLK_SOURCE  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_GPIO        GPIOB
#define I2C1_SDA_GPIO_AF     GPIO_AF_4
#define I2C1_SDA_PIN         GPIO_Pin_9
#define I2C1_SDA_PIN_SOURCE  GPIO_PinSource9
#define I2C1_SDA_CLK_SOURCE  RCC_AHBPeriph_GPIOB
//
#define USE_SPI
//
#define USE_SPI_DEVICE_2
#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI2_NSS_PIN            GPIO_Pin_5
#define SPI2_NSS_PIN_SOURCE     GPIO_PinSource5
#define SPI2_SCK_PIN            GPIO_Pin_13
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
#define SPI2_MISO_PIN           GPIO_Pin_14
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
#define SPI2_MOSI_PIN           GPIO_Pin_15
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15

#define USE_ADC

#define ADC_INSTANCE         ADC2
#define ADC_DMA_CHANNEL      DMA2_Channel1
#define ADC_AHB_PERIPHERAL   RCC_AHBPeriph_DMA2

#define BOARD_HAS_VOLTAGE_DIVIDER

#define VBAT_ADC_GPIO        GPIOB	//TODO: check Vbat working, connected to internal ADC
#define VBAT_ADC_GPIO_PIN    GPIO_Pin_2
#define VBAT_ADC_CHANNEL     ADC_Channel_12


//#define BLACKBOX
#define SERIAL_RX
//#define GPS
//#define GTUNE
//#define DISPLAY
//#define USE_SERVOS
//#define USE_MIXER_SERVOS
//#define USE_CLI

//#define SPEKTRUM_BIND
// USART2, PA3
//#define BIND_PORT  GPIOA
//#define BIND_PIN   Pin_3

#define USE_QUAD_MIXER_ONLY

// alternative defaults for AlienWii32 F3 target
//#define ALIENWII32
//#define HARDWARE_BIND_PLUG

// Hardware bind plug at PB12 (Pin 25)
//#define BINDPLUG_PORT  GPIOB
//#define BINDPLUG_PIN   Pin_12

#define DCM
//#define OPTIC_FLOW

//#define LASER_TOF // only to get laser height
//#define LASER_ALT // to integrate with althold



#define FLIGHT_STATUS_INDICATOR
#define ENABLE_ACROBAT
#define LED_ENABLE
#define TEST_ENABLE

#ifdef __cplusplus
}
#endif 
