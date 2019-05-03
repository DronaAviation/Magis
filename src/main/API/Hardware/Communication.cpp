///*
// * This file is part of Magis.
// *
// * Magis is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * Magis is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with this software.  If not, see <http://www.gnu.org/licenses/>.
// */
//
//#if defined(PRIMUSX)
//
//#include "Communication.h"
//
//#include <stdbool.h>
//#include <stdint.h>
//
//#include "platform.h"
//#include "build_config.h"
//#include "debug.h"
//
//#include "common/maths.h"
//#include "common/axis.h"
//#include "common/utils.h"
//
//#include "drivers/system.h"
//#include "drivers/serial.h"
//#include "drivers/serial_uart.h"
//#include "drivers/gpio.h"
//#include "drivers/light_led.h"
//
//#include "sensors/sensors.h"
//
//#include "io/serial.h"
//#include "io/display.h"
//#include "io/gps.h"
//
//#include "flight/pid.h"
//#include "flight/gps_conversion.h"
//#include "flight/navigation.h"
//
//#include "API/Debug/Print.h"
//
//#include "config/config.h"
//#include "config/runtime_config.h"
//
//#define DISABLE_SPI       GPIO_SetBits(GPIOB,   GPIO_Pin_12)
//#define ENABLE_SPI        GPIO_ResetBits(GPIOB, GPIO_Pin_12)
//
//static serialPort_t* uart2;
//static serialPort_t* uart3;
//static serialPort_t* unibusUARTPort;
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
//void SPIClass::Init()
//{
//    spiInit (SPI2);
//}
//
//void SPIClass::Settings(SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit)
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
//void SPIClass::Settings(SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit,
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
//void SPIClass::Enable(void)
//{
//    ENABLE_SPI;
//}
//
//void SPIClass::Disable(void)
//{
//    DISABLE_SPI;
//}
//
//uint8_t SPIClass::Read(const uint8_t register_address, int len)
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
//bool SPIClass::Write(uint8_t data, const uint8_t register_address)
//{
//    ENABLE_SPI;
//    spiTransferByte(SPI2, register_address);
//    spiTransferByte(SPI2, data);
//    DISABLE_SPI;
//
//    return true;
//}
//
//void I2CClass::Settings(I2Cmode_t mode, I2Cenable_analog_t analog_filter,
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
//
//uint8_t I2CClass::read(uint8_t device_add, uint8_t reg)
//{
//    uint8_t variable = 0;
//    i2cRead(device_add, reg, 1, &variable);
//    return variable;
//}
//
//uint16_t I2CClass::read16(uint8_t device_add, uint8_t reg)
//{
//
//    uint8_t  buffer[2];
//
//    i2cRead(device_add, reg, 2, (uint8_t *)buffer);
//
//    return  ((uint16_t)buffer[0]<<8) | buffer[1];
//}
//
//uint32_t I2CClass::read32(uint8_t device_add, uint8_t reg)
//{
//    uint8_t  buffer[4];
//
//    i2cRead(device_add, reg, 4, (uint8_t *)buffer);
//
//    return  ((uint32_t)buffer[0]<<24) | ((uint32_t)buffer[1]<<16) | ((uint32_t)buffer[2]<<8) | buffer[3];
//}
//
//
//bool I2CClass::write(uint8_t device_add, uint8_t reg, uint8_t data)
//{
//    return i2cWrite(device_add, reg, data);
//}
//
//
//bool I2CClass::write16(uint8_t device_add, uint8_t reg, uint16_t data)
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
//bool I2CClass::write32(uint8_t device_add, uint8_t reg, uint32_t data)
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
//portMode_t mapped_mode;
//portOptions_t mapped_options;
//
////void getMode(UARTportMode_t m, portMode_t* out)
////{
////    switch (m) {
////        case RX:
////            *out = MODE_RX;
////            break;
////
////        case TX:
////            *out = MODE_TX;
////            break;
////
////        case RXTX:
////            *out = MODE_RXTX;
////            break;
////    }
////}
////void getOptions(UARTportOptions_t o, portOptions_t* out)
////{
////    switch (o) {
////        case NOT_INVERTED:
////            *out = SERIAL_NOT_INVERTED;
////            break;
////
////        case INVERTED:
////            *out = SERIAL_INVERTED;
////            break;
////
////        case STOPBITS_1:
////            *out = SERIAL_STOPBITS_1;
////            break;
////
////        case STOPBITS_2:
////            *out = SERIAL_STOPBITS_2;
////            break;
////
////        case PARITY_NO:
////            *out = SERIAL_PARITY_NO;
////            break;
////
////        case PARITY_EVEN:
////            *out = SERIAL_PARITY_EVEN;
////            break;
////
////        case UNIDIR:
////            *out = SERIAL_UNIDIR;
////            break;
////
////        case BIDIR:
////            *out = SERIAL_BIDIR;
////            break;
////    }
////}
//
//void UARTClass::init(uint32_t baud_rate, UARTportMode_e mode,
//        UARTportOptions_e options)
//{
//
//    unibusUARTPort = openSerialPort(SERIAL_PORT_USART2, FUNCTION_UNIBUS, NULL,
//            baud_rate, (portMode_t) mode, (portOptions_t) options);
//
//}
//
//uint8_t UARTClass::read8()
//{
//
//    return serialRead(unibusUARTPort) & 0xff;
//}
//
//uint16_t UARTClass::read16()
//{
//
//    uint16_t t = read8();
//
//    t += (uint16_t) read8() << 8;
//
//    return t;
//}
//
//uint32_t UARTClass::read32()
//{
//
//    uint32_t t = read16();
//
//    t += (uint32_t) read16() << 16;
//
//    return t;
//
//}
//
//void UARTClass::write(uint8_t data)
//{
//
//    serialWrite(unibusUARTPort, data);
//
//}
//
//void UARTClass::write(const char *str)
//{
//
//    serialPrint(unibusUARTPort, str);
//
//}
//
//void UARTClass::write(uint8_t* data, uint16_t length)
//{
//
//    while (length--) {
//
//        serialWrite(unibusUARTPort, *data);
//        data++;
//    }
//
//}
//
//bool UARTClass::rxBytesWaiting()
//{
//
//    return serialRxBytesWaiting(unibusUARTPort);
//}
//
//bool UARTClass::txBytesFree()
//{
//
//    return serialTxBytesFree(unibusUARTPort);
//}
//
//SPIClass Spi;
//I2CClass I2c;
//UARTClass UART;
//
//#endif
