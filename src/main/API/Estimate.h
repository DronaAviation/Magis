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
//#include "Comman.h"

#ifdef __cplusplus
extern "C" {
#endif

class Angle_P {
public:

    //unit: decidegree for ROLL and PITCH
    //unit: degree for YAW

    int16_t get(angle_e ANGLE);

};

class Rate_P {
public:

    //unit: decidegree

    int16_t get(axis_e AXIS);

};

class Position_P {
public:

    //unit: cm

    int16_t get(axis_e AXIS);

};

class Velocity_P {
public:

    //unit: cm/s

    int16_t get(axis_e AXIS);

};

extern Angle_P Angle;
extern Rate_P Rate;
extern Position_P Position;
extern Velocity_P Velocity;

#ifdef __cplusplus
}
#endif
