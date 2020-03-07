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

#include "Control.h"

#include <stdint.h>

#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "blackbox/blackbox.h"

#include "drivers/system.h"
#include "drivers/system.h"
#include "drivers/sensor.h"

#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

#include "io/display.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "command/command.h"

#include "mw.h"

#include "User.h"
#include "API-Utils.h"

int16_t* RcData_P::get(void)
{

    int16_t rcDataArray[11];

    for (int i = 0; i < 11; i++) {

        rcDataArray[i] = rcData[i];

    }

    return rcDataArray;

}

int16_t RcData_P::get(rc_channel_e CHANNEL)
{

    return rcData[CHANNEL];

}

int16_t* RcCommand_P::get(void)
{

    int16_t rcCommandArray[4];

    for (int i = 0; i < 4; i++) {

        rcCommandArray[i] = rcCommand[i];

    }

    return rcCommandArray;

}

int16_t RcCommand_P::get(rc_channel_e CHANNEL)
{

    if (CHANNEL <= RC_THROTTLE) {

        return 1500 + rcCommand[CHANNEL];

    }

    return 0;

}

void RcCommand_P::set(int16_t* rcValueArray)
{

    int16_t rcValue;

    for (int i = 0; i < 4; i++) {

        rcValue = constrain(rcValueArray[i], 1000, 2000);

        if (i < 3) {

            rcValue -= 1500;

            rcCommand[i] = rcValue;

            RC_ARRAY[i] = rcValue;

        } else {

            rcData[i] = rcValue;

            RC_ARRAY[i] = rcValue;

        }

        userRCflag[i] = true;

    }

}

void RcCommand_P::set(rc_channel_e CHANNEL, int16_t rcValue)
{

    int16_t setValue;

    setValue = constrain(rcValue, 1000, 2000);

    if (CHANNEL < RC_THROTTLE) {

        setValue -= 1500;

        rcCommand[CHANNEL] = setValue;

        RC_ARRAY[CHANNEL] = setValue;
        userRCflag[CHANNEL] = true;

    }

    if (CHANNEL == RC_THROTTLE) {

        rcData[CHANNEL] = setValue;

        RC_ARRAY[CHANNEL] = setValue;
        userRCflag[CHANNEL] = true;

    }

}

bool FlightMode_P::check(flight_mode_e MODE)
{

    switch (MODE) {

    case ANGLE:

        return FLIGHT_MODE(ANGLE_MODE);

        break;

    case RATE:

        return !FLIGHT_MODE(ANGLE_MODE);

        break;

    case MAGHOLD:

        return FLIGHT_MODE(MAG_MODE);

        break;

    case HEADFREE:

        return FLIGHT_MODE(HEADFREE_MODE);

        break;

    case ATLTITUDEHOLD:

        return FLIGHT_MODE(BARO_MODE);

        break;

    case THROTTLE_MODE:

        return !FLIGHT_MODE(BARO_MODE);
        break;

    default:

        break;

    }

    return false;

}

void FlightMode_P::set(flight_mode_e MODE)
{

    switch (MODE) {

    case ANGLE:

        ENABLE_FLIGHT_MODE(ANGLE_MODE);
        isUserFlightModeSet[ANGLE] = true;
        isUserFlightModeSet[RATE] = false;

        break;

    case RATE:

        DISABLE_FLIGHT_MODE(ANGLE_MODE);
        isUserFlightModeSet[RATE] = true;
        isUserFlightModeSet[ANGLE] = false;

        break;

    case MAGHOLD:

        ENABLE_FLIGHT_MODE(MAG_MODE);
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        isUserFlightModeSet[MAGHOLD] = true;
        isUserFlightModeSet[HEADFREE] = false;

        break;

    case HEADFREE:

        ENABLE_FLIGHT_MODE(HEADFREE_MODE);
        DISABLE_FLIGHT_MODE(MAG_MODE);

        isUserFlightModeSet[HEADFREE] = true;
        isUserFlightModeSet[MAGHOLD] = false;

        break;

    case ATLTITUDEHOLD:

        ENABLE_FLIGHT_MODE(BARO_MODE);

        isUserFlightModeSet[ATLTITUDEHOLD] = true;
        isUserFlightModeSet[THROTTLE_MODE] = false;

        break;

    case THROTTLE_MODE:

        DISABLE_FLIGHT_MODE(BARO_MODE);

        isUserFlightModeSet[THROTTLE_MODE] = true;
        isUserFlightModeSet[ATLTITUDEHOLD] = false;

        break;

    default:

        break;

    }

}

void Command_P::takeOff(uint16_t height)
{

    if (current_command != TAKE_OFF) {
        current_command = TAKE_OFF;
        command_status = RUNNING;

        height = constrain(height, 100, 250);
        takeOffHeight = height;
    }

}

void Command_P::land(uint8_t landSpeed)
{

    if ((command_status == FINISHED || command_status==ABORT)&&ARMING_FLAG(ARMED)) {
        current_command = LAND;
        command_status = RUNNING;

        landThrottle = 1305 - landSpeed;

    }

}

void Command_P::flip(flip_direction_e direction)
{

    if (current_command != B_FLIP) {
        current_command = B_FLIP;

    }

}

bool Command_P::arm(void)
{

    if (IS_RC_MODE_ACTIVE(BOXARM) && ARMING_FLAG(OK_TO_ARM)) {

        pidResetErrorAngle();
        pidResetErrorGyro();
        mwArm();
    }
    return ARMING_FLAG(ARMED);

}

bool Command_P::disArm(void)
{

    mwDisarm();

    return !ARMING_FLAG(ARMED);

}

flightstatus_e FlightStatus_P::get(void)
{

    return (flightstatus_e) leastSignificantBit(flightIndicatorFlag);

}

bool FlightStatus_P::check(flightstatus_e status)
{

    return status_FSI((FlightStatus_e) status);

}

int16_t App_P::getAppHeading(void)
{

    return appHeading;

}

bool App_P::isArmSwitchOn(void)
{

    return IS_RC_MODE_ACTIVE(BOXARM);

}

void BlackBox_P::setVar(char* varName,int32_t& reference){

#ifdef BLACKBOX
	setUserBlackBoxField(varName,reference);
#endif
}



void setheadFreeModeHeading(int16_t heading)
{

    userHeadFreeHoldHeading = heading;
    isUserHeadFreeHoldSet = true;

}

void setUserLoopFrequency(float frequency)
{

    uint32_t resultFrequency;

    frequency = constrainf(frequency, 3.5, 2000);

    resultFrequency = frequency * 1000;

    userLoopFrequency = resultFrequency;

}

RcData_P RcData;
RcCommand_P RcCommand;
FlightMode_P FlightMode;
Command_P Command;
FlightStatus_P FlightStatus;
App_P App;
BlackBox_P BlackBox;



