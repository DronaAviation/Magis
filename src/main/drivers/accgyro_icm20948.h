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

#pragma once


#ifdef __cplusplus
extern "C" {
#endif

#define ICM20948_GYRO_OUT        0x33
#define ICM20948_ACCEL_OUT         0x2D


bool icm20948AccDetect(acc_t *acc);
bool icm20948GyroDetect(gyro_t *gyro);

void icm20948AccInit(void);
void icm20948GyroInit(uint16_t lpf);




#ifdef __cplusplus
}
#endif
