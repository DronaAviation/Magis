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

class Accelerometer_P {
public:

    int16_t getX(void);
    int16_t getY(void);
    int16_t getZ(void);
    int32_t getNetAcc(void);
};

class Gyroscope_P {
public:

    int16_t getX(void);
    int16_t getY(void);
    int16_t getZ(void);
};

class Magnetometer_P {
public:

    int16_t getX(void);
    int16_t getY(void);
    int16_t getZ(void);
};

class Barometer_P {
public:

    int32_t getPressure(void);
    int32_t getTemperature(void);
};

extern Accelerometer_P Accelerometer;
extern Gyroscope_P Gyroscope;
extern Magnetometer_P Magnetometer;
extern Barometer_P Barometer;

#ifdef __cplusplus
}
#endif
