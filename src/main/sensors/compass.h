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

// Type of magnetometer used/detected
typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_AK8963 = 4,
    MAG_AK09916 = 5
} magSensor_e;

#define MAG_MAX  MAG_AK8963


//int16_t mag_declination;                // Get your magnetic decliniation from here : http://magnetic-declination.com/
// For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.

#ifdef MAG
void compassInit(void);
void updateCompass(flightDynamicsTrims_t *magZero, flightDynamicsTrims_t *magScale);
#endif
void recalculateMagneticDeclination(void);

extern int16_t magADC[XYZ_AXIS_COUNT];
extern uint32_t compassLastUpdatedAt;
extern bool have_initial_yaw;

extern sensor_align_e magAlign;
extern mag_t mag;
extern float magneticDeclination;

#ifdef __cplusplus
}
#endif 
