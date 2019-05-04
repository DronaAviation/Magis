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

typedef struct {

    uint8_t p;
    uint8_t i;
    uint8_t d;

} PID;

typedef enum {

    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_ALT,
    PID_USER

} pid_profile_e;

typedef enum {

    LOW_BATTERY,
    INFLIGHT_LOW_BATTERY,
    CRASH,
    ALL

} failsafe_e;

class DesiredAngle_P {
public:

    //unit: decidegree for ROLL and PITCH
    //unit: degree for YAW

    int32_t get(angle_e ANGLE);

    void set(angle_e ANGLE, int32_t angle);

};

class DesiredRate_P {
public:

    //unit: decidegree

    int32_t get(angle_e ANGLE);

    void set(angle_e ANGLE, int32_t rate);

};

class DesiredPosition_P {
public:

    //unit: cm

    int32_t get(axis_e AXIS);

    void set(axis_e AXIS, int32_t position);

    void setRelative(axis_e AXIS, int32_t position);

};

class DesiredVelocity_P {
public:

    //unit: cm/s

    int32_t get(axis_e AXIS);

    void set(axis_e AXIS, int32_t velocity);

};

class PIDProfile_P {
public:

    void get(pid_profile_e PROFILE, PID* pid);

    void set(pid_profile_e PROFILE, PID* pid);

    void setDefault(void);

};

class Failsafe_P {
public:

    void enable(failsafe_e FAILSAFE);

    void disable(failsafe_e FAILSAFE);

};

extern DesiredAngle_P DesiredAngle;
extern DesiredRate_P DesiredRate;
extern DesiredPosition_P DesiredPosition;
extern DesiredVelocity_P DesiredVelocity;
extern PIDProfile_P PIDProfile;
extern Failsafe_P Failsafe;

#ifdef __cplusplus
}
#endif

