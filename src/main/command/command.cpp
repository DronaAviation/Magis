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
#include "flight/posEstimate.h"
#include "flight/posControl.h"

#include "command/command.h"

#include "../API/Control.h"
#include "../API/Utils.h"
#include "mw.h"


uint8_t current_command = 0;
uint8_t command_status  = 2;

bool isLanding          = false;
bool isArmed            = false;
bool setTakeOffAlt      = false;
bool setTakeOffThrottle = false;
bool setLandTimer       = true;
bool setTakeOffTimer    = true;
bool isTookOff          = false;
bool isTakeOffHeightSet = false;


uint32_t loopTime;
uint32_t takeOffLoopTime;
int32_t takeOffThrottle = 950;
int8_t checkVelocity    = -8;


uint16_t takeOffHeight = 120;
uint16_t landThrottle  = 1200;
bool isUserLandCommand = false;

Interval takeoffTimer;
Interval posSetTimer;



void takeOff ( ) {

  if ( command_status != FINISHED ) {



    if ( ARMING_FLAG ( ARMED ) ) {


      //            if(takeoffTimer.set(1000, false)){



      current_command = NONE;
      command_status  = ABORT;
      isTookOff       = true;

      //            }



    } else {
      if ( IS_RC_MODE_ACTIVE ( BOXARM ) ) {

        pidResetErrorAngle ( );
        pidResetErrorGyro ( );
        mwArm ( );
        takeoffTimer.reset ( );
        posSetTimer.reset ( );

#ifdef LASER_ALT
        if ( ! isTakeOffHeightSet ) {
          DesiredPosition.set ( Z, takeOffHeight );
          isTakeOffHeightSet = true;
        }
#else
        if ( ! isTakeOffHeightSet ) {
          DesiredPosition.setRelative ( Z, takeOffHeight );
          isTakeOffHeightSet = true;
        }
#endif
      }
    }
  }
}
void land ( ) {



  if ( command_status != FINISHED ) {



    isLanding = true;

    if ( setLandTimer ) {

      loopTime     = millis ( ) + 30000;
      setLandTimer = false;
    }

    if ( ( int32_t ) ( millis ( ) - loopTime ) >= 0 ) {

      mwDisarm ( );

      command_status = FINISHED;

      isLanding    = false;
      setLandTimer = true;
    } else {


      if ( ABS ( accADC [ 2 ] ) > 8500 ) {

        command_status = FINISHED;

        mwDisarm ( );

        isLanding         = false;
        setLandTimer      = true;
        isUserLandCommand = false;

        return;
      }



      if ( ABS ( ( int32_t ) ( millis ( ) - loopTime ) ) <= 28000 ) {


        if ( getEstVelocity ( ) > -8 ) {

          command_status = FINISHED;

          mwDisarm ( );

          isLanding         = false;
          setLandTimer      = true;
          isUserLandCommand = false;
        }
      }
    }
  }
}

void executeCommand ( ) {

  switch ( current_command ) {

    case NONE:

      break;

    case TAKE_OFF:

      takeOff ( );

      break;

    case LAND:

      land ( );
      break;

    case B_FLIP:



      if ( flipState == 0 && FLIGHT_MODE ( MAG_MODE ) ) {

        flipDirection = 0;
        flipState     = 1;
        flipStartTime = millis ( );
      }
      current_command = NONE;
      command_status  = FINISHED;



      break;

    case F_FLIP:



      break;

    case R_FLIP:

      break;

    case L_FLIP:


      break;

    default:
      break;
  }
}

void updateCommandStatus ( ) {

  if ( current_command != NONE && command_status == FINISHED ) {

    current_command = NONE;

    command_status = FINISHED;
  }
}
