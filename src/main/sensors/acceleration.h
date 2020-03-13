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

// Type of accelerometer used/detected
typedef enum {
    ACC_DEFAULT = 0,
    ACC_NONE = 1,
    ACC_ADXL345 = 2,
    ACC_MPU6050 = 3,
    ACC_MMA8452 = 4,
    ACC_BMA280 = 5,
    ACC_LSM303DLHC = 6,
    ACC_MPU6000 = 7,
    ACC_MPU6500 = 8,
    ACC_ICM20948=9,
    ACC_FAKE = 10,
} accelerationSensor_e;

#define ACC_MAX  ACC_FAKE

extern sensor_align_e accAlign;
extern acc_t acc;
extern uint16_t acc_1G;

extern int16_t accADC[XYZ_AXIS_COUNT];
extern float accel_offset[XYZ_AXIS_COUNT];
extern float accel_scale[XYZ_AXIS_COUNT];
extern float debug_beta[6];

typedef struct rollAndPitchTrims_s {
        int16_t roll;
        int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union {
        int16_t raw[2];
        rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

bool isAccelerationCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims);
void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse);
void setAccelerationCalibration(flightAccelCalData_T *accelerationCalDataToUse);
flightAccelCalData_T* getAccelerationCalibration();
void calibrate_accel(float trim_roll, float trim_pitch);

#ifdef __cplusplus
}
#endif 
