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

#include "Specifiers.h"

#include <stdint.h>
#include "platform.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h" 

unibus_e unibus;

/*****************PIN FUNCTIONS******************************************************
 PIN		PORT	FUNCTIONS
 1				VBAT
 2		PA13				T4C3			T16_CH1N					debugIO	 IR
 3		PA14	UART2TX		T8C2										debugCLK
 4		PB10	UART3TX		T2C3
 5		PB11	UART3RX		T2C4
 6		PB9		I2C1SDA
 7		PB8		I2C1SCL
 8		PA5					T2C1								A2_IN2			 DAC
 9		PB3		UART2TX**	T2C2			T8C1~						debugSWO
 10		PA8		SPI SS**	T1C1
 11				GND
 12		PB0					T3C3			T1C2~	T8C2~		A3_IN12
 13		PA4					T3C2								A2_IN1			 DAC
 14		PB12	SPI2SS											A4_IN3
 15		PB13	SPI2SCK						T1_C1N				A3_IN5
 16		PB14	SPI2MISO					T1_C2~				A4_IN4
 17		PB15	SPI2MOSI					T1_C3~,T15_C2,1~	A4_IN5
 18		PA3		UART2RX		T2_C4	T15_C2						A1_IN4
 19		PA2		UART2TX		T2_C3	T15_C1						A1_IN3
 20				VCC

 **implementation not possible

 ************************************************************************************/

