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
#include <stdint.h>


#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "platform.h"

#include "serial.h"


void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;

    uint8_t c=0;
    while ((ch = *(str++)) != 0) {
    	//LED_L_ON;
   // 	 lprint("ch:"+ch,2000);
        serialWrite(instance, ch);
        c++;
    }

   // lprint("c:"+c,2000);
    c=0;

}

uint32_t serialGetBaudRate(serialPort_t *instance)
{
    return instance->baudRate;
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}

uint8_t serialRxBytesWaiting(serialPort_t *instance)
{
    return instance->vTable->serialTotalRxWaiting(instance);
}

uint8_t serialTxBytesFree(serialPort_t *instance)
{
    return instance->vTable->serialTotalTxFree(instance);
}

uint8_t serialRead(serialPort_t *instance)
{
    return instance->vTable->serialRead(instance);
}

void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    instance->vTable->serialSetBaudRate(instance, baudRate);
}

bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

void serialSetMode(serialPort_t *instance, portMode_t mode)
{
    instance->vTable->setMode(instance, mode);
}

