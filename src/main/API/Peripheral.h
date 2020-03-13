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

typedef enum {
    INPUT, INPUT_PU, INPUT_PD, OUTPUT,
} GPIO_Mode_e;

typedef enum {
    STATE_LOW, STATE_HIGH, STATE_TOGGLE
} GPIO_State_e;

class GPIO_P {
public:

    void init(unibus_e pin_number, GPIO_Mode_e mode);

    bool read(unibus_e pin_number);

    void write(unibus_e pin_number, GPIO_State_e STATE);

};

class ADC_P {
public:

    void init(unibus_e pin_number);

    uint16_t read(unibus_e pin_number);
};

;

typedef enum {
    UART2, UART3
} UART_Port_e;

typedef enum {
    BAUD_RATE_4800,
    BAUD_RATE_9600,
    BAUD_RATE_14400,
    BAUD_RATE_19200,
    BAUD_RATE_38400,
    BAUD_RATE_57600,
    BAUD_RATE_115200,
    BAUD_RATE_128000,
    BAUD_RATE_256000
} UART_Baud_Rate_e;

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

class I2C_P {
public:

	bool read(uint8_t device_add, uint8_t reg, uint8_t &value);

    int16_t read(uint8_t device_add, uint8_t reg, uint32_t length,uint8_t* buffer);

    bool write(uint8_t device_add, uint8_t reg, uint8_t data);

    bool write(uint8_t device_add, uint8_t reg, uint32_t length, uint8_t* data);

};

class PWM_P {

public:

    void init(unibus_e pin_number, uint16_t pwmRate);

    void write(unibus_e pin_number, uint16_t pwmValue);

};



/*
 MODE 	CPOL	CPHA			Data Captured 		Output
  0		Low		0 i.e.edge1		Rising edge			Falling	edge
  1		Low		1 i.e.edge2		Falling	edge		Rising edge
  2		High	0				Falling	edge		Rising edge
  3		High	1				Rising edge			Falling	edge
*/

typedef enum SPImode_s {
    MODE0 = 0,     //!<SPI mode 1
    MODE1,         //!<SPI mode 2
    MODE2,         //!<SPI mode 3
    MODE3          //!<SPI mode 4
} SPImode_t;

typedef enum SPIfirst_bit_s {
    LSBFIRST = 0,  //!<Specifies that data transfer starts from LSB bit.
    MSBFIRST       //!<Specifies that data transfer starts from MSB bit.
} SPIfirst_bit_t;


class SPI_P {
    public:


        void init();


        void init(SPImode_t mode, uint16_t speed, SPIfirst_bit_t bit);


        void enable(void);


        void disable(void);


        uint8_t read(uint8_t register_address);


        void read(uint8_t register_address, int16_t length,uint8_t* buffer);


        void write(uint8_t register_address,uint8_t data);
};





extern GPIO_P GPIO;
extern ADC_P ADC;
extern UART_P UART;
extern I2C_P I2C;
extern PWM_P PWM;
extern SPI_P SPI;

#ifdef __cplusplus
}
#endif

