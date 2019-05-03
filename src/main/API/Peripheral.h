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

#pragma once

#include <stdint.h>

#include "Specifiers.h"

#ifdef __cplusplus
extern "C" {
#endif

extern bool gpioReset;
extern bool changeAdress;


//typedef enum{
//    AIN,              //!< analog Input
//    IN_FLOATING,      //!<input floating
//    IPD,              //!<input pulled down
//    IPU,              //!<Input with pull-up resistor
//    Out_OD,           //!<output open drain
//    Out_PP,           //!<output push-pull
//    AF_OD,            //!<alternate function open drain
//    AF_PP,            //!<alternate function push pull
//    AF_PP_PD,         //!<alternate function push pull with pull down register
//    AF_PP_PU          //!<alternate function push pull with pull up register
//} GPIO_Mode_e;

typedef enum{
    INPUT,
    INPUT_PU,
    INPUT_PD,
    OUTPUT,
} GPIO_Mode_e;


typedef enum{
    STATE_LOW,
    STATE_HIGH,
    STATE_TOGGLE
} GPIO_State_e;


//typedef enum {
//    SP_10_MHz = 1,
//    SP_2_MHz=2,
//    SP_50_MHz =3
//} GPIO_Speed_e;


//typedef enum GPIOAF_s {
//    AF0, //!<AF0:JTCK-SWCLK, JTDI, JTDO/TRACESW0, JTMS-SWDAT, MCO, NJTRST,TRACED, TRACECK.
//    AF1, //!<AF1:OUT, TIM2, TIM15, TIM16, TIM17.
//    AF2, //!<AF2:COMP1_OUT, TIM1, TIM2, TIM3, TIM4, TIM8, TIM15, TIM16.
//    AF3, //!<AF3:COMP7_OUT, TIM8, TIM15, Touch, HRTIM.
//    AF4, //!<AF4:I2C1, I2C2, TIM1, TIM8, TIM16, TIM17.
//    AF5, //!<AF5:IR_OUT, I2S2, I2S3, SPI1, SPI2, TIM8, USART4, USART5
//    AF6, //!<AF6:IR_OUT, I2S2, I2S3, SPI2, SPI3, TIM1, TIM8
//    AF7, //!< AF7:AOP2_OUT, CAN, COMP3_OUT, COMP5_OUT, COMP6_OUT, USART1,USART2, USART3.
//    AF8, //!< AF8:COMP1_OUT, COMP2_OUT, COMP3_OUT, COMP4_OUT, COMP5_OUT,COMP6_OUT.
//    AF9, //!<AF9:AOP4_OUT, CAN, TIM1, TIM8, TIM15.
//    AF10, //!<AF10:AOP1_OUT, AOP3_OUT, TIM2, TIM3, TIM4, TIM8, TIM17.
//    AF11, //!<AF11:TIM1, TIM8.
//    AF12, //!<AF12:TIM1, HRTIM.
//    AF13, //!< AF13:HRTIM, AOP2_OUT.
//    AF14, //!<AF14:USBDM, USBDP.
//    AF15 //!<AF15:OUT.
//
///**** IMPORTANT: Unibus specific AF mapping is given in the .doc file********/
//} GPIOAF_e;









class GPIO_P {
public:
    /**
     * @brief  Initialize General purpose Input/Output pin
     * @param   unibus_t pin_number: GPIO pin number
     * @param   GPIOmode_t mode:This can be AIN, IN_FLOATING,IPD,IPU,Out_OD,Out_PP,
     *           AF_OD, AF_PP, AF_PP_PD, AF_PP_PU
     * @param   int16_t speed:This can be 2 for 2MHz,10 for 10MHz,50 for 50 MHz
     * @retval None
     */

    void init(unibus_e pin_number, GPIO_Mode_e mode);

    /**
     * @brief Configures alternating functions
     * @param   unibus_t pin_number: GPIO pin number
     * @param   GPIOAF_t AF: Specifies the alternating functions for selected pins
     * @retval None
     */
//    void AFConfig(unibus_e pin_number, GPIOAF_e AF);

    bool read(unibus_e pin_number);

    /**
     * @brief set GPIO pin HIGH
     * @param   unibus_t pin_number: GPIO pin number
     * @retval None
     */
    void write(unibus_e pin_number,GPIO_State_e STATE);


};


class ADC_P {
public:
    /**
     * @brief  Initializes analog to digital conversion
     * @param   unibus_t pin_number: analog pin number
     * @retval None
     */
    void init(unibus_e pin_number);

    /**
     * @brief  Reads the value from the specified pin
     * @param  unibus_t pin_number: analog pin number
     * @retval value from the specified pin
     */
    uint16_t read(unibus_e pin_number);
};


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

typedef enum{
    UART2,
    UART3
}UART_Port_e;


typedef enum{
    BAUD_RATE_4800,
    BAUD_RATE_9600,
    BAUD_RATE_14400,
    BAUD_RATE_19200,
    BAUD_RATE_38400,
    BAUD_RATE_57600,
    BAUD_RATE_115200,
    BAUD_RATE_128000,
    BAUD_RATE_256000
}UART_Baud_Rate_e;


class UART_P {
public:


    void init(UART_Port_e PORT, UART_Baud_Rate_e BAUD);


    uint8_t read8(UART_Port_e PORT);

    uint16_t read16(UART_Port_e PORT);

    uint32_t read32(UART_Port_e PORT);


    void write(UART_Port_e PORT, uint8_t data);

    void write(UART_Port_e PORT, const char *str);

    void write(UART_Port_e PORT, uint8_t* data, uint16_t length);

    bool rxBytesWaiting(UART_Port_e PORT);

    bool txBytesFree(UART_Port_e PORT);

};



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

class I2C_P {
public:


    uint8_t* read(uint8_t device_add, uint8_t reg, uint32_t length);


    bool write(uint8_t device_add, uint8_t reg, uint32_t length, uint8_t* data);

};


//
///* MODE     CPOL    CPHA            Data Captured       Output
// *  0       Low     0 i.e.edge1     Rising edge         Falling edge
// *  1       Low     1 i.e.edge2     Falling edge        Rising edge
// *  2       High    0               Falling edge        Rising edge
// *  3       High    1               Rising edge         Falling edge
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
//
//class SPI_P {
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
//     * @param   SPIdata_size_t size: This parameter can be set to   DataSize_4b,
//     *          DataSize_5b,DataSize_6b,DataSize_7b,DataSize_8b,DataSize_9b,
//     *          DataSize_10b,DataSize_11b,DataSize_12b,DataSize_13b,
//     *          DataSize_14b,DataSize_15b,DataSize_16b
//     * @param   SPIslave_select_t nss: This parameter can be set to Soft,Hard
//     * @param   SPIrole_t role: This parameter can be set to Master,Slave
//     * @param   SPIdata_direction_t direction: This parameter can be set to
//     *          FullDuplex,RxOnly,  Rx, Tx
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
//
//
//class Timer_P {
//public:
//
//    Timer_P();
//
//    void init(unibus_e pin_number, uint16_t hz);
//
//    void setPWM(uint16_t value);
//
//private:
//
//    class TimerImpl;
//    TimerImpl* timerimpl_;
//
//};
//
//class Servo_P {
//
//public:
//
//    void init(unibus_e pin_number);
//
//    void setPWM(uint16_t value);
//
//};


class PWM_P {

public:

    void init(unibus_e pin_number, uint16_t pwmRate);

    void write(unibus_e pin_number, uint16_t pwmValue);

};





extern GPIO_P GPIO;
extern ADC_P ADC;
extern UART_P UART;
extern I2C_P I2C;
extern PWM_P PWM;


#ifdef __cplusplus
}
#endif

