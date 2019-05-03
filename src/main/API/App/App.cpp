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

#include "App.h"

#include <stdint.h>

#include "../API-Utils.h"
#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"



#include "drivers/system.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "io/serial.h"
#include "drivers/gpio.h"



#include "io/gps.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

#include "io/display.h"


#include "telemetry/telemetry.h"


#include "sensors/boardalignment.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"


#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"

#include "blackbox/blackbox.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"


int16_t App_P::getAppHeading(void)
{

    return appHeading;

}

bool App_P::isArmSwitchOn(void)
{

    return IS_RC_MODE_ACTIVE(BOXARM);

}


uint8_t App_P::getUserPID(user_pid_gains_e gain){

   switch (gain) {
    case Kp:

       return  currentProfile->pidProfile.P8[PIDUSER];

        break;

    case Ki:

        return  currentProfile->pidProfile.I8[PIDUSER];

            break;

    case Kd:

        return  currentProfile->pidProfile.D8[PIDUSER];

            break;
    default:

        return 0;
        break;
}

}



void App_P::setUserPID(user_pid_gains_e gain, uint8_t value){

    value=constrain(value, 0, 255);

    switch (gain) {
       case Kp:

          currentProfile->pidProfile.P8[PIDUSER]=value;

           break;

       case Ki:

          currentProfile->pidProfile.I8[PIDUSER]=value;

               break;

       case Kd:

           currentProfile->pidProfile.D8[PIDUSER]=value;

               break;
       default:


           break;

}
}


App_P App;
