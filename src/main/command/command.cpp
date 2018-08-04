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
#include "flight/acrobats.h"

#include "command/command.h"
#include "API/Debug/Utils.h"
#include "API/Debug/Print.h"

#include "mw.h"

#include "API/Flight/Althold.h"
#include "API/Core/Control.h"

uint8_t current_command = 0;
uint8_t command_status = 2;

bool isLanding = false;
bool isArmed = false;
bool setTakeOffAlt = false;
bool setTakeOffThrottle = false;
bool setLandTimer = true;
bool setTakeOffTimer = true;

uint32_t loopTime;
uint32_t takeOffLoopTime;
int32_t takeOffThrottle = 950;
int8_t checkVelocity = -8;

Timer* landingTimer;
Timer* takeOffTimer;

void takeOff()
{


    if (command_status != FINISHED) {

        command_status = RUNNING;

        Print.monitor("##takeoff");

        if (ARMING_FLAG(ARMED)) {
            command_status = FINISHED;

        } else {
            ENABLE_ARMING_FLAG(OK_TO_ARM);
            mwArm();
            Althold.setRelativeAltholdHeight(150);

        }


    } else {


        current_command = NONE;
        command_status = ABORT;

    }

}

void land()
{

    if (command_status != FINISHED) {

        command_status = RUNNING;

        isLanding = true;

        if (setLandTimer) {

            loopTime = millis() + 30000;
            setLandTimer = false;
        }

        if ((int32_t)(millis() - loopTime) >= 0) {
            DEACTIVATE_RC_MODE(BOXARM);
            mwDisarm();
            isLanding = false;
            setLandTimer = true;
        } else {

            if (accADC[2] > 11000) {

                DEACTIVATE_RC_MODE(BOXARM);
                mwDisarm();
                isLanding = false;
                setLandTimer = true;

            }

            if (ABS((int32_t)(millis()-loopTime)) <= 20000 && getSetVelocity() > -8) {

                DEACTIVATE_RC_MODE(BOXARM);

                mwDisarm();

                isLanding = false;
                setLandTimer = true;

            }
        }

    }

}

void executeCommand()
{

    switch (current_command) {

    case NONE:
        break;

    case TAKE_OFF:

        if (command_status == ABORT) {
            pidResetErrorAngle();
            pidResetErrorGyro();
            Print.monitor("##takeoff###");

        }

        takeOff();

        break;

    case LAND:

        land();
        break;

    case BACK_FLIP:


        if (flipState == 0) {

            flipDirection = 0;
            flipState = 1;
            flipStartTime = millis();

        }

        current_command = NONE;
        command_status = ABORT;
        break;

    case FRONT_FLIP:



        break;

    case RIGHT_FLIP:

        break;

    case LEFT_FLIP:


        break;

    default:
        break;

    }

}

void updateCommandStatus()
{

    if (current_command != NONE && command_status == FINISHED) {

        current_command = NONE;

        command_status = ABORT;

    }

}

