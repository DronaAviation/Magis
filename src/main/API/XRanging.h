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

#ifdef __cplusplus
extern "C" {
#endif

typedef enum laser_sensors {
    LEFT = 0,
    RIGHT,
    FRONT,
    BACK,
    EXTERNAL
} laser_e;

class XRanging_P {

public:

    void init(void);
    void init(laser_e laser);
    int16_t getRange(laser_e laser);  // returns range in mm

};

extern XRanging_P XRanging;

#ifdef __cplusplus
}
#endif

