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
//#pragma once
//
//#include <stdint.h>
//
//#include "Specifiers.h"
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
///* MODE 	CPOL	CPHA			Data Captured 		Output
// *	0		Low		0 i.e.edge1		Rising edge			Falling	edge
// *	1		Low		1 i.e.edge2		Falling	edge		Rising edge
// *	2		High	0				Falling	edge		Rising edge
// *	3		High	1				Rising edge			Falling	edge
// */
//typedef enum SPImode_s {
//    MODE0 = 0,     //!<SPI mode 1
//    MODE1,         //!<SPI mode 2
//    MODE2,         //!<SPI mode 3
//    MODE3          //!<SPI mode 4
//} SPImode_t;
//
//typedef enum SPIfirst_bit_s {
//    LSBFIRST = 0,  //!<Specifies that data transfer starts from LSB bit.
//    MSBFIRST       //!<Specifies that data transfer starts from MSB bit.
//} SPIfirst_bit_t;
//
//typedef enum SPIdata_size_s {
//    //Specifies the SPI data size.
//
//    DataSize_4b = 0,
//    DataSize_5b,
//    DataSize_6b,
//    DataSize_7b,
//    DataSize_8b,
//    DataSize_9b,
//    DataSize_10b,
//    DataSize_11b,
//    DataSize_12b,
//    DataSize_13b,
//    DataSize_14b,
//    DataSize_15b,
//    DataSize_16b
//} SPIdata_size_t;
//
//typedef enum SPIslave_select_s {
//    /*! Specifies whether the NSS signal is managed by
//     hardware (NSS pin) or by software using the SSI bit.
//     */
//    Soft = 0, Hard
//} SPIslave_select_t;
//
//typedef enum SPIrole_s {
//    Master = 0,  //!<set the register values for sensor chip as SPI master role
//    Slave        //!slave role to read the sensor data from sensor chip.
//} SPIrole_t;
//
//typedef enum SPIdata_direction_s {
//    //Specifies the SPI unidirectional or bidirectional data mode
//    FullDuplex = 1,
//    RxOnly,
//    Rx,
//    Tx
//} SPIdata_direction_t;
//
//class SPIClass {
//public:
//
//    /**
//     * @brief Initializes SPI protocol
//     * @param   none
//     * @retval None
//     */
//    void Init();
//
//    /**
//     * @brief Configure basic SPI functions
//     * @param  SPImode_t mode: This parameter can be set to MODE0,MODE1,
//     MODE2,MODE3
//     * @param  uint16_t speed:SPI allowed frequencies in MHz:
//     *         18000, 9000, 4500, 2250,
//     *         1125, 562.5, 281.25, 140.625
//     * @param  SPIfirst_bit_t bit:This parameter can be set to LSBFIRST,MSBFIRST
//     * @retval None
//     */
//    void Settings(SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit);
//    /**
//     * @brief Configure STM32F30X settings for all SPI functions
//     * @param  SPImode_t mode: This parameter can be set to MODE0,MODE1,
//     MODE2,MODE3
//     * @param  uint16_t speed:SPI allowed frequencies in MHz:
//     *         18000, 9000, 4500, 2250,
//     *         1125, 562.5, 281.25, 140.625
//     * @param  SPIfirst_bit_t bit:This parameter can be set to LSBFIRST,MSBFIRST
//     * @param   SPIdata_size_t size: This parameter can be set to 	DataSize_4b,
//     *          DataSize_5b,DataSize_6b,DataSize_7b,DataSize_8b,DataSize_9b,
//     *          DataSize_10b,DataSize_11b,DataSize_12b,DataSize_13b,
//     *          DataSize_14b,DataSize_15b,DataSize_16b
//     * @param   SPIslave_select_t nss: This parameter can be set to Soft,Hard
//     * @param   SPIrole_t role: This parameter can be set to Master,Slave
//     * @param   SPIdata_direction_t direction: This parameter can be set to
//     *          FullDuplex,RxOnly,	Rx,	Tx
//     * @param   int crc:Specifies the polynomial used for the CRC calculation.
//     *
//     * @retval None
//     */
//    void Settings(SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit,
//            SPIdata_size_t size, SPIslave_select_t nss, SPIrole_t role,
//            SPIdata_direction_t direction, int crc);
//
//    /**
//     * @brief Enables SPI communication
//     * @param   none
//     * @retval None
//     */
//    void Enable(void);
//
//    /**
//     * @brief Disables SPI communication
//     * @param   none
//     * @retval None
//     */
//    void Disable(void);
//
//    /**
//     * @brief Reads the values from the specified register
//     * @param   const uint8_t register_address:This parameter can be set to
//     *           0x00 to 0xFF
//     * @param   int lenght:number of bytes. min value:1
//     * @retval value from the specified register
//     */
//    uint8_t Read(const uint8_t register_address, int length);
//
//    /**
//     * @brief Reads the values from the specified register
//     * @param   const uint8_t data:value to be written to the register
//     * @param   const uint8_t register_address:This parameter can be set to
//     *          0x00 to 0xFF
//     * @retval  bool : Either TRUE/FALSE
//     */
//    bool Write(uint8_t data, const uint8_t register_address);
//};
//
//typedef enum I2Cmode_s {
//
//    //Specifies the I2C mode.
//    I2Cmode,
//    SMBusDevice,
//    SMBusHost
//} I2Cmode_t;
//
//typedef enum I2Cenable_analog_s {
//    AnalogFilter_Enable, //!<Enables analog noise filter.
//    AnalogFilter_Disable //!< Disables analog noise filter.
//} I2Cenable_analog_t;
//
//typedef enum I2Cack_flag_s {
//    Ack_Enable,    //!<Enables the acknowledgement
//    Ack_Disable    //!<Disables the acknowledgement
//} I2Cack_flag_t;
//
//typedef enum I2Cack_address_s {
//    AckAddress_7bit,   //!<Specifies 7-bit address is acknowledged
//    AckAddress_10bit   //!Specifies 10-bit address is acknowledged
//} I2Cack_address_t;
//
//class I2CClass {
//public:
//
//    /**
//     * @brief Configure STM32F30X settings for all I2C functions
//     * @param  I2Cmode_t mode: This parameter can be set to I2Cmode,SMBusDevice,
//     *          SMBusHost
//     * @param   I2Cenable_analog_t analog_filter:This parameter can be set to
//     *          AnalogFilter_Enable,AnalogFilter_Disable
//     * @param   uint32_t digital_filter: Configures the digital noise filter.This
//     parameter can be a number between 0x00 and 0x0F
//     * @param   uint32_t address: This parameter can have value 0x00 to 0xFF
//     * @param   I2Cack_flag_t ack: This parameter can be set to
//     *          Ack_Enable,Ack_Disable
//     * @param   I2Cack_address_t ack_address: This parameter can be set to
//     *          AckAddress_7bit,	AckAddress_10bit
//     * @param   uint32_t timing:Specifies the I2C_TIMINGR_register value.This
//     parameter calculated by referring to I2C initialization
//     section in Reference manual
//     * @retval None
//     */
//    void Settings(I2Cmode_t mode, I2Cenable_analog_t analog_filter,
//            uint32_t digital_filter, uint32_t address, I2Cack_flag_t ack,
//            I2Cack_address_t ack_address, uint32_t timing);
//
//    /**
//     * @brief   Reads the values from the specified location
//     * @param   uint8_t device_add:This parameter is the address
//     *          of the device can be set from 0x00 to 0xFF
//     * @param   uint8_t reg: This parameter can be set from 0x00 to 0xFF
//     * @param   uint8_t lenght:number of bytes. min value:1
//     * @retval  value from the specified location
//     */
//    uint8_t read(uint8_t device_add, uint8_t reg);
//
//
//    uint16_t read16(uint8_t device_add, uint8_t reg);
//
//
//    uint32_t read32(uint8_t device_add, uint8_t reg);
//
//   // uint8_t Read(uint8_t device_add, uint8_t reg, uint8_t length);
//
//
//    /**
//     * @brief Writes the value to the specified location
//     * @param  uint8_t device_add:This parameter is the address
//     *          of the device can be set from 0x00 to 0xFF
//     * @param    uint8_t reg:This parameter can be set to
//     *          0x00 to 0xFF
//     * @param    uint8_t data:value to be written to the register
//     * @retval  bool : Either TRUE/FALSE
//     */
//    bool write(uint8_t device_add, uint8_t reg, uint8_t data);
//    bool write16(uint8_t device_add, uint8_t reg, uint16_t data);
//    bool write32(uint8_t device_add, uint8_t reg, uint32_t data);
//};
//
//typedef enum UARTportMode_s {
//
//    //Specifies wether the Receive or Transmit mode is enabled or disabled.
//    RX = 1 << 0,
//    TX = 1 << 1,
//    RXTX = RX | TX
//} UARTportMode_e;
//
//typedef enum UARTportOptions_s {
//    NOT_INVERTED = 0 << 0,    //!< 0 V represents 0 and 3.3 V represents 1.
//    INVERTED = 1 << 0,        //!<  0 V represents 1 and 3.3 V represents 0
//    STOPBITS_1 = 0 << 1, //!<Stop bits sent at the end of every character allow the receiving
//    STOPBITS_2 = 1 << 1, //!< signal hardware to detect the end of a character and to
//                         //!<resynchronize with the character stream.
//    PARITY_NO = 0 << 2,       //!< no parity
//    PARITY_EVEN = 1 << 2,     //!< even parity
//    UNIDIR = 0 << 3,          //!< Unidirectional
//    BIDIR = 1 << 3            //!< Bidirectional
//} UARTportOptions_e;
//
//class UARTClass {
//public:
//
//    /**
//     * @brief Initializes UART protocol
//     * @param  unibus_t TXpin_number:transmitter pin number
//     * @param  unibus_t RXpin_number:receiver pin number
//     * @retval None
//     */
//    void init(uint32_t baud_rate, UARTportMode_e mode,
//            UARTportOptions_e options);
//
//    /**
//     * @brief Configure STM32F30X settings for all UART functions
//     * @param  unibus_t pin_number : pin number
//     * @param  uint32_t baud_rate: 0, 9600, 19200, 38400,
//     *         57600, 115200, 230400, 250000
//     * @param   UARTportMode_t mode: This parameter can be set to RX,TX, RXTX
//     * @param   UARTportOptions_t options: This parameter can be set to
//     *          NOT_INVERTED, INVERTED, STOPBITS_1, STOPBITS_2, PARITY_NO,
//     *          PARITY_EVEN, UNIDIR,BIDIR
//     * @retval None
//     */
//    //  void Settings(unibus_t pin_number,);
//    /**
//     * @brief Reads the value from the specified location
//     * @param  unibus_t pin_number: pin number
//     * @retval value from the specified location
//     */
//    uint8_t read8();
//
//    uint16_t read16();
//
//    uint32_t read32();
//
//    /**
//     * @brief  writes to the specified location
//     * @param  unibus_t pin_number: pin number
//     * @param  unit8_t data: data to be written
//     * @retval bool: Either TRUE/FALSE
//     */
//    void write(uint8_t data);
//
//    void write(const char *str);
//
//    void write(uint8_t* data, uint16_t length);
//
//    bool rxBytesWaiting(void);
//
//    bool txBytesFree(void);
//
//};
//
//extern UARTClass UART;
//extern I2CClass I2c;
//extern SPIClass Spi;
//
//#ifdef __cplusplus
//}
//#endif
//
