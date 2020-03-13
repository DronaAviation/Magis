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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "exti.h"
#include "gpio.h"
#include "bus_i2c.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_icm20948.h"


extern uint16_t acc_1G;
extern uint8_t mpuLowPassFilter;

bool icm20948AccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_ICM_20948) {
        return false;
    }

    acc->init = icm20948AccInit;
    acc->read = mpuAccRead;

    return true;
}

bool icm20948GyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_ICM_20948) {
        return false;
    }

    gyro->init = icm20948GyroInit;
    gyro->read = mpuGyroRead;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

void icm20948AccInit(void)
{
    mpuIntExtiInit();

    acc_1G = 512 * 8;
}

void icm20948GyroInit(uint16_t lpf)
{
    mpuIntExtiInit();


    bool ack=false;

    mpuConfiguration.write(0x7F, 0x20);

    delay(20);

    mpuConfiguration.write(0x01, (0x06 | (0b00011000 | 0x01))); //gyro lpf and rate

    delay(10);


    mpuConfiguration.write(0x00, 0x00); //gyro sample rate

    delay(10);


    mpuConfiguration.write(0x14, (0x04 | ( 0b00110000 | 0x01))); //acc lpf and rate

    delay(10);


    mpuConfiguration.write(0x10, 0x00); //acc sample rate higher byte

    delay(10);


    mpuConfiguration.write(0x11, 0x00); //acc sample rate lower byte




}
