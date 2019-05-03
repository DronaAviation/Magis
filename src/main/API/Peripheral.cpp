/*
 * This file is part of Magis.
 *
 * Magis is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Magis is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(PRIMUSX)

#include "Peripheral.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"
#include "build_config.h"
#include "common/utils.h"
#include "common/atomic.h"

#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "drivers/timer_stm32f30x.h"
#include "drivers/timer_impl.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"



#include "sensors/sensors.h"

#include "io/serial.h"
#include "io/display.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/gps_conversion.h"
#include "flight/navigation.h"

#include "API/Debug/Print.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "User.h"
#include "Control.h"
#include "XRanging.h"
#include "Utils.h"


#include "API-Utils.h"


bool gpioReset = false;
bool changeAdress = false;


pwmOutputPort_t* pwm[11];



//
///*Mapping function*/
//
//uint8_t getGPIOaf(GPIOAF_e AF)
//{
//    uint8_t af = 0;
//    switch (AF) {
//    case AF0:
//        af = GPIO_AF_0;
//        break;
//
//    case AF1:
//        af = GPIO_AF_1;
//        break;
//
//    case AF2:
//        af = GPIO_AF_2;
//        break;
//
//    case AF3:
//        af = GPIO_AF_3;
//        break;
//
//    case AF4:
//        af = GPIO_AF_4;
//        break;
//
//    case AF5:
//        af = GPIO_AF_5;
//        break;
//
//    case AF6:
//        af = GPIO_AF_6;
//        break;
//
//    case AF7:
//        af = GPIO_AF_7;
//        break;
//
//    case AF8:
//        af = GPIO_AF_8;
//        break;
//
//    case AF9:
//        af = GPIO_AF_9;
//        break;
//
//    case AF10:
//        af = GPIO_AF_10;
//        break;
//
//    case AF11:
//        af = GPIO_AF_11;
//        break;
//
//    case AF12:
//        af = GPIO_AF_12;
//        break;
//
//    case AF13:
//        af = GPIO_AF_13;
//        break;
//
//    case AF14:
//        af = GPIO_AF_14;
//        break;
//
//    case AF15:
//        af = GPIO_AF_15;
//        break;
//    }
//    return af;
//}

void GPIO_P::init(unibus_e pin_number, GPIO_Mode_e mode)
{
//    int gpioX;
  //  GPIO_Pin pinY;

//    gpioX = getGPIOport(pin_number);
//    pinY = getGPIOpin(pin_number);
//    clockZ = getGPIOclock(pin_number);

    uint32_t clock;
    GPIO_TypeDef* gpio;
    gpio_config_t cfg;


//    if (gpioX == 1) {
//        gpio = GPIOA;
//    } else if (gpioX == 2) {
//        gpio = GPIOB;
//    }
//
//    cfg.pin = pinY;


    switch(pin_number){

    case Pin2:

        gpio=GPIOA;
        cfg.pin = Pin_13;
        clock = RCC_AHBPeriph_GPIOA;

        break;

    case Pin3:

        gpio=GPIOA;
        cfg.pin = Pin_14;
        clock = RCC_AHBPeriph_GPIOA;

        break;

    case Pin6:

        gpio=GPIOB;
        cfg.pin = Pin_9;
        clock = RCC_AHBPeriph_GPIOB;

        break;


    case Pin7:

        gpio=GPIOB;
        cfg.pin = Pin_8;
        clock = RCC_AHBPeriph_GPIOB;

        break;


    case Pin8:

        gpio=GPIOA;
        cfg.pin = Pin_5;
        clock = RCC_AHBPeriph_GPIOA;

        break;


    case Pin9:

        gpio=GPIOB;
        cfg.pin = Pin_3;
        clock = RCC_AHBPeriph_GPIOB;

        break;


    case Pin10:

        gpio=GPIOA;
        cfg.pin = Pin_8;
        clock = RCC_AHBPeriph_GPIOA;

        break;


    case Pin13:

        gpio=GPIOA;
        cfg.pin = Pin_4;
        clock = RCC_AHBPeriph_GPIOA;

        break;


    case Pin14:

        gpio=GPIOB;
        cfg.pin = Pin_12;
        clock = RCC_AHBPeriph_GPIOB;

        break;


    case Pin15:

        gpio=GPIOB;
        cfg.pin = Pin_13;
        clock = RCC_AHBPeriph_GPIOB;

        break;


    case Pin16:

        gpio=GPIOB;
        cfg.pin = Pin_14;
        clock = RCC_AHBPeriph_GPIOB;

        break;


    case Pin17:

        gpio=GPIOB;
        cfg.pin = Pin_15;
        clock = RCC_AHBPeriph_GPIOB;

        break;


    case Pin18:

        gpio=GPIOA;
        cfg.pin = Pin_3;
        clock = RCC_AHBPeriph_GPIOA;

        break;


    case Pin19:

        gpio=GPIOA;
        cfg.pin = Pin_2;
        clock = RCC_AHBPeriph_GPIOA;

        break;

    }



    switch (mode) {
    case INPUT:
        cfg.mode = Mode_IN_FLOATING;
        break;

    case INPUT_PU:
        cfg.mode = Mode_IPU;
        break;

    case INPUT_PD:
        cfg.mode = Mode_IPD;
        break;

    case OUTPUT:
        cfg.mode = Mode_Out_PP;
        break;

    }

    cfg.speed = Speed_2MHz;

    RCC_AHBPeriphClockCmd(clock, ENABLE);

    gpioInit(gpio, &cfg);
}
//
//void GPIO_P::AFConfig(unibus_e pin_number, GPIOAF_e AF)
//{
//    int gpioX;
//    uint8_t pinSrcY;
//    uint8_t af;
//
//    gpioX = getGPIOport(pin_number);
//    pinSrcY = getGPIOpinSource(pin_number);
//    af = getGPIOaf(AF);
//
//    GPIO_TypeDef* gpio;
//    if (gpioX == 1) {
//        gpio = GPIOA;
//    } else if (gpioX == 2) {
//        gpio = GPIOB;
//    }
//
//    gpioAFConfig(gpio, pinSrcY, af);
//}

bool GPIO_P::read(unibus_e pin_number)
{

    uint16_t pin;
    GPIO_TypeDef* gpio;

//    int gpioX = getGPIOport(pin_number);
//
//    if (gpioX == 1) {
//        gpio = GPIOA;
//    } else if (gpioX == 2) {
//        gpio = GPIOB;
//    }

    switch(pin_number){

    case Pin2:

        gpio=GPIOA;
        pin = Pin_13;


        break;

    case Pin3:

        gpio=GPIOA;
        pin = Pin_14;

        break;

    case Pin6:

        gpio=GPIOB;
        pin = Pin_9;


        break;


    case Pin7:

        gpio=GPIOB;
        pin = Pin_8;


        break;


    case Pin8:

        gpio=GPIOA;
        pin = Pin_5;

        break;


    case Pin9:

        gpio=GPIOB;
        pin = Pin_3;


        break;


    case Pin10:

        gpio=GPIOA;
        pin = Pin_8;


        break;


    case Pin13:

        gpio=GPIOA;
        pin = Pin_4;


        break;


    case Pin14:

        gpio=GPIOB;
        pin = Pin_12;


        break;


    case Pin15:

        gpio=GPIOB;
        pin = Pin_13;

        break;


    case Pin16:

        gpio=GPIOB;
        pin = Pin_14;


        break;


    case Pin17:

        gpio=GPIOB;
        pin = Pin_15;


        break;


    case Pin18:

        gpio=GPIOA;
        pin = Pin_3;


        break;


    case Pin19:

        gpio=GPIOA;
        pin = Pin_2;


        break;

    }





    return digitalIn(gpio, pin);

}


void GPIO_P::write(unibus_e pin_number, GPIO_State_e STATE)
{

//    GPIO_TypeDef* gpio;
//
//    int gpioX = getGPIOport(pin_number);
//
//    if (gpioX == 1) {
//        gpio = GPIOA;
//    } else if (gpioX == 2) {
//        gpio = GPIOB;
//    }


    uint16_t pin;
    GPIO_TypeDef* gpio;

//    int gpioX = getGPIOport(pin_number);
//
//    if (gpioX == 1) {
//        gpio = GPIOA;
//    } else if (gpioX == 2) {
//        gpio = GPIOB;
//    }

    switch(pin_number){

    case Pin2:

        gpio=GPIOA;
        pin = Pin_13;


        break;

    case Pin3:

        gpio=GPIOA;
        pin = Pin_14;

        break;

    case Pin6:

        gpio=GPIOB;
        pin = Pin_9;


        break;


    case Pin7:

        gpio=GPIOB;
        pin = Pin_8;


        break;


    case Pin8:

        gpio=GPIOA;
        pin = Pin_5;

        break;


    case Pin9:

        gpio=GPIOB;
        pin = Pin_3;


        break;


    case Pin10:

        gpio=GPIOA;
        pin = Pin_8;


        break;


    case Pin13:

        gpio=GPIOA;
        pin = Pin_4;


        break;


    case Pin14:

        gpio=GPIOB;
        pin = Pin_12;


        break;


    case Pin15:

        gpio=GPIOB;
        pin = Pin_13;

        break;


    case Pin16:

        gpio=GPIOB;
        pin = Pin_14;


        break;


    case Pin17:

        gpio=GPIOB;
        pin = Pin_15;


        break;


    case Pin18:

        gpio=GPIOA;
        pin = Pin_3;


        break;


    case Pin19:

        gpio=GPIOA;
        pin = Pin_2;


        break;

    }


    switch(STATE){

    case STATE_LOW:

        digitalLo(gpio, pin);
        break;


    case STATE_HIGH:

        digitalHi(gpio, pin);
        break;


    case STATE_TOGGLE:

        digitalToggle(gpio, pin);
        break;
    }
}



void ADC_P::init(unibus_e pin_number)
{



    switch (pin_number) {

    case Pin8:

        isADCEnable[UB_ADC2_IN2]= true;

        break;

    case Pin13:

        isADCEnable[UB_ADC2_IN1]= true;
        break;

    case Pin14:

        isADCEnable[UB_ADC4_IN3]= true;
        break;

    case Pin15:

        isADCEnable[UB_ADC3_IN5]= true;
        break;

    case Pin16:

        isADCEnable[UB_ADC4_IN4]= true;
        break;

    case Pin17:

        isADCEnable[UB_ADC4_IN5]= true;
        break;

    case Pin18:

        isADCEnable[UB_ADC1_IN4]= true;
        break;

    case Pin19:

        isADCEnable[UB_ADC1_IN3]= true;
        break;

    default:

        break;

    }


}

uint16_t ADC_P::read(unibus_e pin_number)
{

    switch (pin_number) {

    case Pin8:

        return adc2Values[adcDmaIndex[UB_ADC2_IN2]];

        break;

    case Pin13:

        return adc2Values[adcDmaIndex[UB_ADC2_IN1]];
        break;

    case Pin14:

        return adc4Values[adcDmaIndex[UB_ADC4_IN3]];
        break;

    case Pin15:

        return adc3Values[adcDmaIndex[UB_ADC3_IN5]];
        break;

    case Pin16:

        return adc4Values[adcDmaIndex[UB_ADC4_IN4]];
        break;

    case Pin17:

        return adc4Values[adcDmaIndex[UB_ADC4_IN5]];
        break;

    case Pin18:

        return adc1Values[adcDmaIndex[UB_ADC1_IN4]];
        break;

    case Pin19:

        return adc1Values[adcDmaIndex[UB_ADC1_IN3]];
        break;

    default:

        return 0;
        break;

    }

}

static serialPort_t* uart2;
static serialPort_t* uart3;
//static serialPort_t* unibusUARTPort;


portMode_t mapped_mode;
portOptions_t mapped_options;



//void getMode(UARTportMode_t m, portMode_t* out)
//{
//    switch (m) {
//        case RX:
//            *out = MODE_RX;
//            break;
//
//        case TX:
//            *out = MODE_TX;
//            break;
//
//        case RXTX:
//            *out = MODE_RXTX;
//            break;
//    }
//}
uint32_t getBaud(UART_Baud_Rate_e BAUD)
{
    switch (BAUD) {

        case BAUD_RATE_4800:

            return 4800;

            break;

        case BAUD_RATE_9600:

            return 9600;

            break;

        case BAUD_RATE_14400:

            return 14400;

            break;

        case BAUD_RATE_19200:

            return 19200;

            break;

        case BAUD_RATE_38400:

            return 38400;

            break;

        case BAUD_RATE_57600:

            return 57600;

            break;

        case BAUD_RATE_115200:

            return 115200;

            break;

        case BAUD_RATE_128000:

            return 128000;

            break;

        case BAUD_RATE_256000:

            return 256000;

            break;
    }
}

void UART_P::init(UART_Port_e PORT, UART_Baud_Rate_e BAUD)
{


    if(PORT==UART2){

        uart2 = openSerialPort(SERIAL_PORT_USART2, FUNCTION_UNIBUS, NULL,
                getBaud(BAUD), MODE_RXTX, SERIAL_NOT_INVERTED);

    }else if(PORT==UART3){

        uart3 = openSerialPort(SERIAL_PORT_USART3, FUNCTION_UNIBUS, NULL,
                getBaud(BAUD), MODE_RXTX, SERIAL_NOT_INVERTED);


    }
}

uint8_t UART_P::read8(UART_Port_e PORT)
{



    if(PORT==UART2){

        return serialRead(uart2) & 0xff;

    }else if(PORT==UART3){

        return serialRead(uart3) & 0xff;

    }
}

uint16_t UART_P::read16(UART_Port_e PORT)
{

    uint16_t t = read8(PORT);

    t += (uint16_t) read8(PORT) << 8;

    return t;
}

uint32_t UART_P::read32(UART_Port_e PORT)
{

    uint32_t t = read16(PORT);

    t += (uint32_t) read16(PORT) << 16;

    return t;

}

void UART_P::write(UART_Port_e PORT, uint8_t data)
{

    if(PORT==UART2){

        serialWrite(uart2, data);

    }else if(PORT==UART3){

        serialWrite(uart3, data);

    }

}

void UART_P::write(UART_Port_e PORT, const char *str)
{


    if(PORT==UART2){

        serialPrint(uart2, str);

    }else if(PORT==UART3){

        serialPrint(uart3, str);

    }

}

void UART_P::write(UART_Port_e PORT, uint8_t* data, uint16_t length)
{


    if(PORT==UART2){

        while (length--) {

            serialWrite(uart2, *data);
            data++;
        }

    }else if(PORT==UART3){

        while (length--) {

            serialWrite(uart3, *data);
            data++;
        }

    }


}

bool UART_P::rxBytesWaiting(UART_Port_e PORT)
{


    if(PORT==UART2){

        return serialRxBytesWaiting(uart2);

    }else if(PORT==UART3){

        return serialRxBytesWaiting(uart3);

    }
}

bool UART_P::txBytesFree(UART_Port_e PORT)
{

    if(PORT==UART2){

        return serialTxBytesFree(uart2);

    }else if(PORT==UART3){

        return serialTxBytesFree(uart3);
    }
}



//void I2C_P::Settings(I2Cmode_t mode, I2Cenable_analog_t analog_filter,
//        uint32_t digital_filter, uint32_t address, I2Cack_flag_t ack,
//        I2Cack_address_t ack_address, uint32_t timing)
//{
//    I2C_InitTypeDef I2C_InitStructure;
//
//    I2C_StructInit(&I2C_InitStructure);
//
//    switch (mode) {
//    case I2Cmode:
//        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//        break;
//
//    case SMBusDevice:
//        I2C_InitStructure.I2C_Mode = I2C_Mode_SMBusDevice;
//        break;
//
//    case SMBusHost:
//        I2C_InitStructure.I2C_Mode = I2C_Mode_SMBusHost;
//        break;
//    }
//    if (analog_filter == AnalogFilter_Enable) {
//        I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
//    } else if (analog_filter == AnalogFilter_Disable) {
//        I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Disable;
//    }
//    I2C_InitStructure.I2C_DigitalFilter = digital_filter;
//    I2C_InitStructure.I2C_OwnAddress1 = address;
//    if (ack == Ack_Enable) {
//        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//    } else if (ack == Ack_Disable) {
//        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//    }
//    if (ack_address == AckAddress_7bit) {
//        I2C_InitStructure.I2C_AcknowledgedAddress =
//                I2C_AcknowledgedAddress_7bit;
//    } else if (ack_address == AckAddress_10bit) {
//        I2C_InitStructure.I2C_AcknowledgedAddress =
//                I2C_AcknowledgedAddress_10bit;
//    }
//    I2C_InitStructure.I2C_Timing = timing;
//
//    I2C_Init(I2C1, &I2C_InitStructure);
//
//    I2C_Cmd(I2C1, ENABLE);
//}

uint8_t* I2C_P::read(uint8_t device_add, uint8_t reg, uint32_t length)
{
    uint8_t  buffer[length];

    i2cRead(device_add, reg, length, (uint8_t *)buffer);


    return buffer;
}

//uint16_t I2C_P::read16(uint8_t device_add, uint8_t reg)
//{
//
//    uint8_t  buffer[2];
//
//    i2cRead(device_add, reg, 2, (uint8_t *)buffer);
//
//    return  ((uint16_t)buffer[0]<<8) | buffer[1];
//}
//
//uint32_t I2C_P::read32(uint8_t device_add, uint8_t reg)
//{
//    uint8_t  buffer[4];
//
//    i2cRead(device_add, reg, 4, (uint8_t *)buffer);
//
//    return  ((uint32_t)buffer[0]<<24) | ((uint32_t)buffer[1]<<16) | ((uint32_t)buffer[2]<<8) | buffer[3];
//}


bool I2C_P::write(uint8_t device_add, uint8_t reg, uint32_t length, uint8_t* data)
{
    bool status;
    status=i2cWriteBuffer(device_add, reg, length, data);
    return status;
}

//
//bool I2C_P::write16(uint8_t device_add, uint8_t reg, uint16_t data)
//{
//
//   uint8_t  buffer[2];
//   bool status;
//
//   // Split 16-bit word into MS and LS uint8_t
//   buffer[1] = (uint8_t)(data & 0xFF);
//   buffer[0] = (uint8_t)((data >> 8) & 0xFF);
//
//   status=i2cWriteBuffer(device_add, reg, 2, (uint8_t *)buffer);
//
//
//   return status;
//}
//
//
//bool I2C_P::write32(uint8_t device_add, uint8_t reg, uint32_t data)
//{
//
//    uint8_t  buffer[4];
//    bool status;
//
//    // Split 32-bit word into MS ... LS bytes
//    buffer[3] = (uint8_t) (data &  0xFF);
//    buffer[2] = (uint8_t) ((data >> 8) & 0xFF);
//    buffer[0] = (uint8_t) ((data >> 24) & 0xFF);
//    buffer[1] = (uint8_t) ((data >> 16) & 0xFF);
//
//    status=i2cWriteBuffer(device_add, reg, 4, (uint8_t *)buffer);
//
//
//    return status;
//}


//
//#define DISABLE_SPI       GPIO_SetBits(GPIOB,   GPIO_Pin_12)
//#define ENABLE_SPI        GPIO_ResetBits(GPIOB, GPIO_Pin_12)
//
//
//
//uint16_t getPrescaler(uint16_t speed)
//{
//    uint16_t prescaler = 0;
//    switch (speed) {
//    case 140:
//        prescaler = SPI_BaudRatePrescaler_256;
//        break;
//
//    case 281:
//        prescaler = SPI_BaudRatePrescaler_128;
//        break;
//
//    case 562:
//        prescaler = SPI_BaudRatePrescaler_64;
//        break;
//
//    case 1125:
//        prescaler = SPI_BaudRatePrescaler_32;
//        break;
//
//    case 2250:
//        prescaler = SPI_BaudRatePrescaler_16;
//        break;
//
//    case 4500:
//        prescaler = SPI_BaudRatePrescaler_8;
//        break;
//
//    case 9000:
//        prescaler = SPI_BaudRatePrescaler_4;
//        break;
//
//    case 18000:
//        prescaler = SPI_BaudRatePrescaler_2;
//        break;
//    }
//    return prescaler;
//}
//
//uint16_t getDataSize(SPIdata_size_t size)
//{
//    uint16_t sizebits = 0;
//    switch (size) {
//    case DataSize_4b:
//        sizebits = SPI_DataSize_4b;
//        break;
//
//    case DataSize_5b:
//        sizebits = SPI_DataSize_5b;
//        break;
//
//    case DataSize_6b:
//        sizebits = SPI_DataSize_6b;
//        break;
//
//    case DataSize_7b:
//        sizebits = SPI_DataSize_7b;
//        break;
//
//    case DataSize_8b:
//        sizebits = SPI_DataSize_8b;
//        break;
//
//    case DataSize_9b:
//        sizebits = SPI_DataSize_9b;
//        break;
//
//    case DataSize_10b:
//        sizebits = SPI_DataSize_10b;
//        break;
//
//    case DataSize_11b:
//        sizebits = SPI_DataSize_11b;
//        break;
//
//    case DataSize_12b:
//        sizebits = SPI_DataSize_12b;
//        break;
//
//    case DataSize_13b:
//        sizebits = SPI_DataSize_13b;
//        break;
//
//    case DataSize_14b:
//        sizebits = SPI_DataSize_14b;
//        break;
//
//    case DataSize_15b:
//        sizebits = SPI_DataSize_15b;
//        break;
//
//    case DataSize_16b:
//        sizebits = SPI_DataSize_16b;
//        break;
//    }
//    return sizebits;
//}
//
//void SPI_P::Init()
//{
//    spiInit (SPI2);
//}
//
//void SPI_P::Settings(SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit)
//{
//    uint16_t prescaler = getPrescaler(speed);
//    SPI_InitTypeDef spi;
//
//    SPI_I2S_DeInit (SPI2);
//
//    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    spi.SPI_Mode = SPI_Mode_Master;
//    spi.SPI_DataSize = SPI_DataSize_8b;
//    switch (mode) {
//    case MODE0:
//        spi.SPI_CPOL = SPI_CPOL_Low;
//        spi.SPI_CPHA = SPI_CPHA_1Edge;
//        break;
//
//    case MODE1:
//        spi.SPI_CPOL = SPI_CPOL_Low;
//        spi.SPI_CPHA = SPI_CPHA_2Edge;
//        break;
//
//    case MODE2:
//        spi.SPI_CPOL = SPI_CPOL_High;
//        spi.SPI_CPHA = SPI_CPHA_1Edge;
//        break;
//
//    case MODE3:
//        spi.SPI_CPOL = SPI_CPOL_High;
//        spi.SPI_CPHA = SPI_CPHA_2Edge;
//        break;
//    }
//    spi.SPI_NSS = SPI_NSS_Soft;
//    spi.SPI_BaudRatePrescaler = prescaler;
//    if ((bit == LSBFIRST)) {
//        spi.SPI_FirstBit = SPI_FirstBit_LSB;
//    } else if ((bit == MSBFIRST)) {
//        spi.SPI_FirstBit = SPI_FirstBit_MSB;
//    }
//    spi.SPI_CRCPolynomial = 7;
//
//#ifdef STM32F303xC
//    // Configure for 8-bit reads.
//    SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
//#endif
//    SPI_Init(SPI2, &spi);
//    SPI_Cmd(SPI2, ENABLE);
//
//    // Drive NSS high to disable connected SPI device.
//    DISABLE_SPI;
//}
//
//void SPI_P::Settings(SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit,
//        SPIdata_size_t size, SPIslave_select_t nss, SPIrole_t role,
//        SPIdata_direction_t direction, int crc)
//{
//    uint16_t prescaler = getPrescaler(speed);
//    uint16_t sizebits = getDataSize(size);
//
//    SPI_InitTypeDef spi;
//
//    SPI_I2S_DeInit (SPI2);
//
//    switch (direction) {
//    case FullDuplex:
//        spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//        break;
//
//    case RxOnly:
//        spi.SPI_Direction = SPI_Direction_2Lines_RxOnly;
//        break;
//
//    case Rx:
//        spi.SPI_Direction = SPI_Direction_1Line_Rx;
//        break;
//
//    case Tx:
//        spi.SPI_Direction = SPI_Direction_1Line_Tx;
//        break;
//    }
//    if (role == Master) {
//        spi.SPI_Mode = SPI_Mode_Master;
//    } else if (role == Slave) {
//        spi.SPI_Mode = SPI_Mode_Slave;
//    }
//    spi.SPI_DataSize = sizebits;
//    switch (mode) {
//    case MODE0:
//        spi.SPI_CPOL = SPI_CPOL_Low;
//        spi.SPI_CPHA = SPI_CPHA_1Edge;
//        break;
//
//    case MODE1:
//        spi.SPI_CPOL = SPI_CPOL_Low;
//        spi.SPI_CPHA = SPI_CPHA_2Edge;
//        break;
//
//    case MODE2:
//        spi.SPI_CPOL = SPI_CPOL_High;
//        spi.SPI_CPHA = SPI_CPHA_1Edge;
//        break;
//
//    case MODE3:
//        spi.SPI_CPOL = SPI_CPOL_High;
//        spi.SPI_CPHA = SPI_CPHA_2Edge;
//        break;
//    }
//    if ((nss == Soft)) {
//        spi.SPI_NSS = SPI_NSS_Soft;
//    } else if ((nss == Hard)) {
//        spi.SPI_NSS = SPI_NSS_Hard;
//    }
//    spi.SPI_BaudRatePrescaler = prescaler;
//    if ((bit == LSBFIRST)) {
//        spi.SPI_FirstBit = SPI_FirstBit_LSB;
//    } else if ((bit == MSBFIRST)) {
//        spi.SPI_FirstBit = SPI_FirstBit_MSB;
//    }
//    spi.SPI_CRCPolynomial = crc; ///TODO crc?
//
//#ifdef STM32F303xC
//    // Configure for 8-bit reads.
//    SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
//#endif
//    SPI_Init(SPI2, &spi);
//    SPI_Cmd(SPI2, ENABLE);
//
//    // Drive NSS high to disable connected SPI device.
//    DISABLE_SPI;
//}
//
//void SPI_P::Enable(void)
//{
//    ENABLE_SPI;
//}
//
//void SPI_P::Disable(void)
//{
//    DISABLE_SPI;
//}
//
//uint8_t SPI_P::Read(const uint8_t register_address, int len)
//{
//    uint8_t variable = 10;
//
//    ENABLE_SPI;
//    spiTransferByte(SPI2, register_address);
//    spiTransfer(SPI2, &variable, NULL, len);
//    DISABLE_SPI;
//
//    return variable;
//
//}
//
//bool SPI_P::Write(uint8_t data, const uint8_t register_address)
//{
//    ENABLE_SPI;
//    spiTransferByte(SPI2, register_address);
//    spiTransferByte(SPI2, data);
//    DISABLE_SPI;
//
//    return true;
//}


//
//class Timer_P::TimerImpl {
//
//public:
//    pwmOutputPort_t * pwmOutPort;
//
//};
//
//
//Timer_P::Timer_P()
//{
//
//}
//
//
//void Timer_P::init(unibus_e pin_number, uint16_t hz)
//{
//
//    timerHardware_t timerHardware = { TIM4, GPIOA, Pin_13, TIM_Channel_3,TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_10 };
//
//    GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);
//
//    uint32_t ghz = PWM_BRUSHED_TIMER_MHZ * 1000000;
//
//    this->timerimpl_ = new TimerImpl;
//    this->timerimpl_->pwmOutPort = pwmOutConfig(&timerHardware, 1, 1000000 / hz, 1000);
//
//}
//
//void Timer_P::setPWM(uint16_t value)
//{
//
//    *(this->timerimpl_->pwmOutPort->ccr) = value;
//
//}
//
//
//void Servo_P::init(unibus_e pin_number)
//{
////
////    timerHardware_t timerHardware = { TIM4, GPIOA, Pin_13, TIM_Channel_3,TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_10 };
////
////    GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);
////
////    uint32_t ghz = PWM_BRUSHED_TIMER_MHZ * 1000000;
////
////    this->timerimpl_ = new TimerImpl;
////    this->timerimpl_->pwmOutPort = pwmOutConfig(&timerHardware, 1, 1000000 / 50, 1000);
//
//}
//
//
//void Servo_P::setPWM(uint16_t value, uint16_t hz)
//{
//
////    *(this->timerimpl_->pwmOutPort->ccr) = value;
//
//}
//

void PWM_P::init(unibus_e pin_number, uint16_t pwmRate){

    timerHardware_t timerHardware;
    uint32_t hz;


    switch (pin_number) {

    case Pin2:

        if(!isPwmInit[0]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);


         timerHardware = { TIM4, GPIOA, Pin_13, TIM_Channel_3,TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_10 };

         GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

         hz = PWM_TIMER_MHZ * 1000000;

         pwm[0] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

         isPwmInit[0]=true;

        }

         break;

    case Pin3:

        if(!isPwmInit[1]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);


        timerHardware = { TIM8, GPIOA, Pin_14, TIM_Channel_2,TIM8_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_5 };

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[1] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[1]=true;

        }

        break;

    case Pin4:

        if(!isPwmInit[2]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


        timerHardware = { TIM2, GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1 };

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[2] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[2]=true;

        }

        break;

    case Pin5:

        if(!isPwmInit[3]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);



        timerHardware = { TIM2, GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1 };

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[3] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[3]=true;

        }

        break;

    case Pin8:

        if(!isPwmInit[4]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);



        timerHardware = { TIM2, GPIOA, Pin_5, TIM_Channel_1, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource5, GPIO_AF_1};

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[4] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[4]=true;

        }

        break;

    case Pin9:


        if(!isPwmInit[5]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


        timerHardware = { TIM2, GPIOB, Pin_3, TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_1 };

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[5] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[5]=true;

        }

        break;

    case Pin10:

        if(!isPwmInit[6]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);


        timerHardware = { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource8, GPIO_AF_6 };

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[6] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[6]=true;

        }

        break;

    case Pin13:

        if(!isPwmInit[7]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);



        timerHardware = {TIM3, GPIOA, Pin_4, TIM_Channel_2, TIM3_IRQn, 0, Mode_AF_PP, GPIO_PinSource4, GPIO_AF_2 };

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[7] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[7]=true;

        }

        break;
/*
    case Pin15:

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);



        timerHardware = {TIM1, GPIOB, Pin_13, TIM_Channel_11, TIM1_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_6};

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[8] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[8]=true;

        break;
*/

    case Pin18:


        if(!isPwmInit[9]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


        timerHardware = { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_1 };

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[9] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[9]=true;

        }

        break;


    case Pin19:


        if(!isPwmInit[10]){

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


        timerHardware = {TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_1 };

        GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

        hz = PWM_TIMER_MHZ * 1000000;

        pwm[10] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

        isPwmInit[10]=true;

        }

        break;




    }


}

void PWM_P::write(unibus_e pin_number, uint16_t pwmValue){


    pwmValue=constrain(pwmValue,500,2500);

    switch (pin_number) {

    case Pin2:

        if(isPwmInit[0]){

            *pwm[0]->ccr=pwmValue;

        }

         break;

    case Pin3:

        if(isPwmInit[1]){

            *pwm[1]->ccr=pwmValue;

        }

        break;

    case Pin4:

        if(isPwmInit[2]){

            *pwm[2]->ccr=pwmValue;

        }

        break;

    case Pin5:

        if(isPwmInit[3]){

            *pwm[3]->ccr=pwmValue;

        }

        break;

    case Pin8:

        if(isPwmInit[4]){

            *pwm[4]->ccr=pwmValue;

        }

        break;

    case Pin9:

        if(isPwmInit[5]){

            *pwm[5]->ccr=pwmValue;

        }

        break;

    case Pin10:

        if(isPwmInit[6]){

            *pwm[6]->ccr=pwmValue;

        }

        break;

    case Pin13:

        if(isPwmInit[7]){

            *pwm[7]->ccr=pwmValue;

        }

        break;
/*
    case Pin15:

        if(isPwmInit[8]){

            *pwm[8]->ccr=pwmValue;

        }

        break;
*/
    case Pin18:

        if(isPwmInit[9]){

            *pwm[9]->ccr=pwmValue;

        }

        break;


    case Pin19:

        if(isPwmInit[10]){

            *pwm[10]->ccr=pwmValue;

        }

        break;


    default:


        break;

    }



}





GPIO_P GPIO;
ADC_P ADC;
UART_P UART;
I2C_P I2C;
PWM_P PWM;



#endif
