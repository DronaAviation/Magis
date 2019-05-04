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

#pragma once

#include <stdint.h>
#include "common/axis.h"
#include "Comman.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {

    PRESSURE, TEMPERATURE

} baro_state_e;

class Accelerometer_P {
public:

    //unit: cm/sec2

    int16_t get(axis_e AXIS);
    int32_t getNetAcc(void);

};

class Gyroscope_P {
public:

    //unit: decidegree/sec

    int16_t get(axis_e AXIS);

};

class Magnetometer_P {
public:

    //unit: microTesla

    int16_t get(axis_e AXIS);

};

class Barometer_P {
public:

    //unit: 100*millibar for Pressure
    //unit" 100*degreeCelsius Temperature

    int32_t get(baro_state_e STATE);

};

extern Accelerometer_P Acceleration;
extern Gyroscope_P Gyroscope;
extern Magnetometer_P Magnetometer;
extern Barometer_P Barometer;

#ifdef __cplusplus
}
#endif
