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
#include "accgyro_icm20689.h"

#include "drivers/light_led.h"

extern uint16_t acc_1G;
extern uint8_t mpuLowPassFilter;

bool ICM20689AccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != ICM_20689) {
        return false;
    }

    acc->init = ICM20689AccInit;
    acc->read = mpuAccRead;

    return true;
}

bool ICM20689GyroDetect(gyro_t *gyro)
{

    if (mpuDetectionResult.sensor != ICM_20689) {
        return false;
    }


    gyro->init = ICM20689GyroInit;
    gyro->read = mpuGyroRead;

    // 16.4 dps/lsb scalefactor
    gyro->scale = 1.0f / 16.4f;

    return true;
}

void ICM20689AccInit(void)
{
    //mpuIntExtiInit();

    acc_1G = 512 * 8;
}


void ICM20689GyroInit(uint16_t lpf)
{
    //mpuIntExtiInit();


    uint8_t mpuLowPassFilter = determineMPULPF(lpf);

    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, ICM20689_BIT_RESET);	//Device reset
    delay(100);
    //mpuConfiguration.write(MPU_RA_SIGNAL_PATH_RESET, 0x03);			//Reset acc & temp digital path
    //delay(100);
    mpuConfiguration.write(MPU_RA_USER_CTRL, 0x01);				//Reset gyro digital path
    delay(100);
    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0);				//Sleep mode off, Internal oscillator 20Mhz
    delay(100);
    mpuConfiguration.write(MPU_RA_PWR_MGMT_2, 0);				//Enable acc, gyro
    delay(100);
    //mpuConfiguration.write(MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    mpuConfiguration.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delay(10);
    mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    delay(10);
    mpuConfiguration.write(MPU_RA_CONFIG, mpuLowPassFilter); //Filter Gyro with masterConfig.gyro_lpf (41Hz)
    delay(10);
    mpuConfiguration.write(0x1D, 0x04);	// Acc low pass filter of 21.2Hz
    delay(10);
    mpuConfiguration.write(MPU_RA_SMPLRT_DIV, 0); // 1kHz S/R
    delay(10);

    // Data ready interrupt configuration
    //mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0); // INT_ANYRD_2CLEAR, BYPASS_EN
#ifdef USE_MPU_DATA_READY_SIGNAL
    mpuConfiguration.write(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable
#endif

}
