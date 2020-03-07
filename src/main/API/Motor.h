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

typedef enum motor {

    M1 = 0,
    M2,
    M3,
    M4,
    M5,
    M6,
    M7,
    M8

} motor_e;

typedef enum {

    CLOCK_WISE = 0,
    ANTICLOCK_WISE

} motor_aerial_direction_e;

typedef enum {

    FORWARD = 0,
    BACKWARD

} motor_terrestrial_direction_e;

class Motor_P {

public:

    void init(motor_e motor);
    void initReversibleMotors(void);
    void set(motor_e motor, int16_t pwmValue);
    void setDirection(motor_e motor, motor_aerial_direction_e direction);
    void setDirection(motor_e motor, motor_terrestrial_direction_e direction);

};

extern Motor_P Motor;

#ifdef __cplusplus
}
#endif

