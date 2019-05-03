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

#include "../API-Utils.h"
#include "../Hardware/Led.h"
#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "drivers/system.h"
#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
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

#include "blackbox/blackbox.h"

#include "command/command.h"

#include "mw.h"


bool Control_P::isOkToArm()
{

    return ARMING_FLAG(OK_TO_ARM);
}

bool Control_P::isArmed()
{

    return ARMING_FLAG(ARMED);

}

bool Control_P::arm()
{
    if (IS_RC_MODE_ACTIVE(BOXARM) && ARMING_FLAG(OK_TO_ARM)) {

        pidResetErrorAngle();
        pidResetErrorGyro();
        mwArm();
    }
    return ARMING_FLAG(ARMED);

}

bool Control_P::disArm()
{

    mwDisarm();

    return !ARMING_FLAG(ARMED);
}


void Control_P::setRcCommand(rc_channels_e channel, int16_t value)
{

    value = constrain(value, 1000, 2000);


    switch(channel){

    case RC_ROLL:
        value-=1500;

           rcCommand[channel]=value;
           RC_ARRAY[channel]=value;
           userRCflag[channel]=true;
           break;



    case RC_PITCH:
            value-=1500;

               rcCommand[channel]=value;
               RC_ARRAY[channel]=value;
               userRCflag[channel]=true;
               break;

    case RC_YAW:
            value-=1500;

               rcCommand[channel]=value;
               RC_ARRAY[channel]=value;
               userRCflag[channel]=true;
               break;


    case RC_THROTTLE:
        rcData[channel]=value;
        RC_ARRAY[channel]=value;
        userRCflag[channel]=true;

               break;


    }

}

int16_t Control_P::getRcData(rc_channels_e channel)
{



    switch(channel){

    case RC_ROLL:
        return rcData[channel];
           break;



    case RC_PITCH:
        return rcData[channel];
               break;

    case RC_YAW:

            return rcData[channel];
               break;


    case RC_THROTTLE:
        return rcData[channel];
               break;

    case RC_USER1:
           return rcData[channel+4];
                  break;

   case RC_USER2:

               return rcData[channel+4];
                  break;


   case RC_USER3:
           return rcData[channel+4];
                  break;

    }



}

void Control_P::disableFlightStatus(bool disable)
{
    if (disable)
        FlightStatusEnabled = false;
    else
        FlightStatusEnabled = true;



}

//void Control_P::disableExternalRC(rc_channels_e channel)
//{
//
//    External_RC_FLAG[channel] = false;
//
//}
//
//void Control_P::disableAllExternalRC(void)
//{
//
//    for (int channel = 0; channel < 4; channel++) {
//        External_RC_FLAG[channel] = false;
//
//    }
//
//}
//
//void Control_P::enableExternalRC(rc_channels_e channel)
//{
//
//    External_RC_FLAG[channel] = true;
//
//}
//
//void Control_P::enableAllExternalRC(void)
//{
//
//    for (int channel = 0; channel < 4; channel++) {
//        External_RC_FLAG[channel] = true;
//
//    }
//
//}

bool Control_P::checkFlightStatus(f_status_e status)
{

    return status_FSI((FlightStatus_e) status);

}

f_status_e Control_P::getCurrentFlightStatus(void)
{

    return (f_status_e) leastSignificantBit(flightIndicatorFlag);

}

void Control_P::setFailsafeState(failsafe_e failsafe, bool active)
{

    switch (failsafe) {

    case LOW_BATTERY:

        fsLowBattery = active;
        break;

    case INFLIGHT_LOW_BATTERY:

        fsInFlightLowBattery = active;
        break;

    case CRASH:
        fsCrash = active;
        break;

    case ALL:
        fsLowBattery = active;
        fsInFlightLowBattery = active;
        fsCrash = active;
        break;

    }

}

void Control_P::setCommand(flight_command_e command)
{

    if(current_command!=command){
    current_command = command;
    command_status = RUNNING;
    }

}

void Control_P::setHeading(int16_t heading)
{
    magHold = heading;
    userHeading = heading;
    isUserHeadingSet = true;


}

void Control_P::setUserLoopFrequency(float frequency)
{
    uint32_t resultFrequency;

    frequency = constrain(frequency, 3.5, 2000);

    resultFrequency = frequency * 1000;

    userLoopFrequency = resultFrequency;

}

void Control_P::enableDeveloperMode(void)
{

    developerMode=true;

}

void Control_P::disableDeveloperMode(void)
{

    developerMode=false;

}

Control_P Control;

