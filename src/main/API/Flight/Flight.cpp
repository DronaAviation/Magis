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

#include "Flight.h"

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

#include "drivers/gpio.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"

#include "rx/rx.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

#include "io/display.h"


#include "telemetry/telemetry.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "config/config.h"
#include "config/runtime_config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

 modeActivationCondition_t *condition;

//int16_t Flight_P::getAngle(angles_e angle)
//{
//
//    switch (angle) {
//
//    case AG_ROLL:
//        return inclination.values.rollDeciDegrees;
//        break;
//
//    case AG_PITCH:
//        return inclination.values.pitchDeciDegrees;
//        break;
//
//    case AG_YAW:
//        return heading;
//        break;
//
//    default:
//
//        break;
//    }
//
//}



int32_t Flight_P::getEstimatedAltitude(void)
{
    return altitudeHoldGetEstimatedAltitude();

}

int32_t Flight_P::getVelocityZ(void)
{

    return getSetVelocity();
}

void Flight_P::activateFlightMode(flight_mode_e mode)
{

    switch (mode) {

      case FM_ALTHOLD:
         // AUX3_VALUE=1500;
          ACTIVATE_RC_MODE(BOXBARO);
          break;

      case FM_ANGLE:
//          condition=&currentProfile->modeActivationConditions[1];
//          condition->range.startStep=0;
//          writeEEPROM();
//          readEEPROM();
         ENABLE_FLIGHT_MODE(ANGLE_MODE);
          break;

      default:

          break;
      }

}

void Flight_P::deactivateFlightMode(flight_mode_e mode)
{

    switch (mode) {

      case FM_ALTHOLD:
        //  AUX3_VALUE=1200;
          DEACTIVATE_RC_MODE(BOXBARO);

          break;

      case FM_ANGLE:
//          condition=&currentProfile->modeActivationConditions[1];
//          condition->range.startStep=48;
//          writeEEPROM();
//          readEEPROM();
          DISABLE_FLIGHT_MODE(ANGLE_MODE);
          break;

      default:

          break;
      }

}




bool Flight_P::isFlightMode(flight_mode_e mode)
{

    switch (mode) {

         case FM_ALTHOLD:
             return IS_RC_MODE_ACTIVE(BOXBARO);
             break;

         case FM_ANGLE:
             return FLIGHT_MODE(ANGLE_MODE);
             break;

         default:

             break;
         }

    return false;
}
//
//void Flight_P::setAngleRate(angles_e angle, uint8_t rate){
//
//    switch (angle) {
//
//       case AG_ROLL:
//           currentControlRateProfile->rates[FD_ROLL]=rate;
//           break;
//
//       case AG_PITCH:
//           currentControlRateProfile->rates[FD_PITCH]=rate;
//           break;
//
//       case AG_YAW:
//           currentControlRateProfile->rates[FD_YAW]=rate;
//           break;
//
//       default:
//
//           break;
//       }
//
//
//
//}

//uint8_t Flight_P::getAngleRate(angles_e angle){
//
//    switch (angle) {
//
//       case AG_ROLL:
//          return currentControlRateProfile->rates[FD_ROLL];
//           break;
//
//       case AG_PITCH:
//           return currentControlRateProfile->rates[FD_PITCH];
//           break;
//
//       case AG_YAW:
//           return currentControlRateProfile->rates[FD_YAW];
//           break;
//
//       default:
//
//           break;
//       }
//
//      return 0;
//
//}


void Flight_P::setAltholdHeight(int32_t height)
{
  setUserAltitude(height);

}

void Flight_P::setRelativeAltholdHeight(int32_t height)
{
 setRelativeUserAltitude(height);

}

int32_t Flight_P::getAltholdHeight(void)
{
 return AltHold;

}

Flight_P Flight;

