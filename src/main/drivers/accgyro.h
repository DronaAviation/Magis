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

extern uint16_t acc_1G; // FIXME move into acc_t

typedef struct gyro_s {
        sensorGyroInitFuncPtr init;                             // initialize function
        sensorReadFuncPtr read;                                 // read 3 axis data function
        sensorReadFuncPtr temperature;                          // read temperature if available
        float scale;                                            // scalefactor
} gyro_t;

typedef struct acc_s {
        sensorInitFuncPtr init;                                 // initialize function
        sensorReadFuncPtr read;                                 // read 3 axis data function
        char revisionCode;                                      // a revision code for the sensor, if known
} acc_t;

#ifdef __cplusplus
}
#endif 
