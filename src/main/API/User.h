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

typedef enum {

    RC_ROLL = 0,
    RC_PITCH,
    RC_YAW,
    RC_THROTTLE,
    RC_AUX1,
    RC_AUX2,
    RC_AUX3,
    RC_AUX4,
    RC_USER1,
    RC_USER2,
    RC_USER3

} rc_channel_e;

typedef enum {
    ANGLE, RATE, MAGHOLD, HEADFREE, ATLTITUDEHOLD, THROTTLE_MODE
} flight_mode_e;

typedef enum {

    BACK_FLIP

} flip_direction_e;

typedef enum {

    FS_ACCEL_GYRO_CALIBRATION = 0,
    FS_MAG_CALIBARATION,
    FS_LOW_BATTERY,
    FS_INFLIGHT_LOW_BATTERY,
    FS_CRASHED,
    FS_SIGNAL_LOSS,
    FS_NOT_OK_TO_ARM,
    FS_OK_TO_ARM,
    FS_ARMED

} flightstatus_e;

class RcData_P {
public:

    int16_t* get(void);
    int16_t get(rc_channel_e CHANNEL);

};

class RcCommand_P {
public:

    int16_t* get(void);
    int16_t get(rc_channel_e CHANNEL);
    void set(int16_t* rcValueArray);
    void set(rc_channel_e CHANNEL, int16_t rcValue);

};

class FlightMode_P {
public:

    bool check(flight_mode_e MODE);
    void set(flight_mode_e MODE);

};

class Command_P {
public:

    void takeOff(uint16_t height = 150);
    void land(uint8_t landSpeed = 105);
    void flip(flip_direction_e direction);
    bool arm(void);
    bool disArm(void);

};

class FlightStatus_P {
public:

    flightstatus_e get(void);
    bool check(flightstatus_e status);

};

class App_P {
public:

    int16_t getAppHeading(void);
    bool isArmSwitchOn(void);

};



class BlackBox_P{

public:
	void setVar(char* varName, int32_t& reference);


};





void setheadFreeModeHeading(int16_t heading);

void setUserLoopFrequency(float frequency);  //frequency is in milliseconds

extern RcData_P RcData;
extern RcCommand_P RcCommand;
extern FlightMode_P FlightMode;
extern Command_P Command;
extern FlightStatus_P FlightStatus;
extern App_P App;
extern BlackBox_P BlackBox;
#ifdef __cplusplus
}
#endif

