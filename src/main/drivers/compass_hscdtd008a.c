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

#include <math.h>

#include "build_config.h"

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "gpio.h"
#include "bus_i2c.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#include "sensor.h"
#include "compass.h"

#include "compass_hscdtd008a.h"
#include "drivers/light_led.h"		//temp for debugging

// This sensor is available in MPU-9150.

// AK8975, mag sensor address
#define HSCDTD_MAG_I2C_ADDRESS      0x0c

// Registers
#define HSCDTD_MAG_REG_WHO_AM_I     0x0f
#define HSCDTD_MAG_CONTROL1     	0x1b
#define HSCDTD_MAG_CONTROL2     	0x1c
#define HSCDTD_MAG_CONTROL3     	0x1d
#define HSCDTD_MAG_CONTROL4     	0x1e
#define HSCDTD_MAG_STATUS	     	0x18
#define HSCDTD_MAG_OUTPUT_X_OUT		0x10

#define BIT_STATUS_REG_DATA_READY			(1 << 6)
#define BIT_STATUS_REG_MAG_SENSOR_OVERRUN	(1 << 5)

bool hscdtdDetect(mag_t *mag)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(HSCDTD_MAG_I2C_ADDRESS, HSCDTD_MAG_REG_WHO_AM_I, 1, &sig);
    if (!ack || sig != 0x49)
        return false;

    mag->init = hscdtdInit;
    mag->read = hscdtdRead;

    return true;
}


void hscdtdInit()
{
    bool ack;
    uint8_t buffer[3];
    uint8_t status;

    UNUSED(ack);

    ack = i2cWrite(HSCDTD_MAG_I2C_ADDRESS, HSCDTD_MAG_CONTROL4, 0x90); // 15-bit data
    delay(10);
    ack = i2cWrite(HSCDTD_MAG_I2C_ADDRESS, HSCDTD_MAG_CONTROL1, 0x88); // Active mode, 10hz odr, continuous measurement
    delay(10);


}


bool hscdtdRead(int16_t *magData)
{
    bool ack;
    UNUSED(ack);
    uint8_t status;
    uint8_t buf[6];

    ack = i2cRead(HSCDTD_MAG_I2C_ADDRESS, HSCDTD_MAG_STATUS, 1, &status);
    if (!ack || (status & BIT_STATUS_REG_DATA_READY) == 0) {
        return false;
    }


    /*ack = i2cRead(HSCDTD_MAG_I2C_ADDRESS, HSCDTD_MAG_STATUS, 1, &status);
    if (status & BIT_STATUS_REG_MAG_SENSOR_OVERRUN) {
            return false;
        }*/

    ack = i2cRead(HSCDTD_MAG_I2C_ADDRESS, HSCDTD_MAG_OUTPUT_X_OUT, 6, buf); // read from AK8975_MAG_REG_HXL to AK8975_MAG_REG_HZH

    if (!ack) {
            return false;
        }



    magData[X] = -(int16_t) (buf[1] << 8 | buf[0]);
    magData[Y] = -(int16_t) (buf[3] << 8 | buf[2]);
    magData[Z] = -(int16_t) (buf[5] << 8 | buf[4]);


    return true;
}
