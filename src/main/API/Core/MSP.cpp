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

#include "MSP.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"

#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/system.h"

#include "io/serial_msp.h"


#define MSP_API_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_API_SET_COMMAND          217    //in message          command

static uint8_t msp_checksum = 0;

void msp_write8(uint8_t a)
{

    serialize8Debug(a);
    msp_checksum ^= a;

}

void msp_write16(uint16_t a)
{

    msp_write8((uint8_t)(a >> 0));
    msp_write8((uint8_t)(a >> 8));
}

void msp_write32(uint32_t a)
{

    msp_write16((uint8_t)(a >> 0));
    msp_write16((uint8_t)(a >> 16));

}

void mspSendHeader(uint8_t mspCommand, uint8_t bodySize)
{

    msp_write8('$');
    msp_write8('M');
    msp_write8('<');
    msp_checksum = 0;               // start calculating a new checksum
    msp_write8(bodySize);
    msp_write8(mspCommand);

}

void mspSendTail()
{

    msp_write8(msp_checksum);
}

void MSP_P::setRC(int16_t rcChannels[])
{

    mspSendHeader(MSP_API_SET_RAW_RC, 16);

    for (int i = 0; i < 8; i++)
        msp_write16(rcChannels[i]);

    mspSendTail();

}

void MSP_P::setCommand(int16_t command)
{

    mspSendHeader(MSP_API_SET_COMMAND, 2);


    msp_write16(command);

    mspSendTail();

}

