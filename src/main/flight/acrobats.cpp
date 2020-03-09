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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "common/axis.h"
#include "common/maths.h"
#include "drivers/light_led.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"
#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "telemetry/telemetry.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "acrobats.h"

#include "API/Sensor.h"


uint8_t flipDirection = 0;
int16_t pitch = 0;
int16_t roll = 0;
uint32_t flipState = 0;
uint32_t flipStartTime;
uint32_t stateon = 0;
const int32_t desiredVelocity = 100;

bool takeControl = false;
bool isPitchStabelised = false;
bool isRollStabelised = false;

void flip(bool doFlip)
{
    if (doFlip) {
        switch (flipState) {


     //   rcData[THROTTLE] = 1800;

        case ASCEND: //State 1


//
//            LED_L_ON;
//            LED_M_OFF;
//            LED_R_OFF;



            if ((millis() - flipStartTime) > 2200) {
                ACTIVATE_RC_MODE(BOXBARO);
                flipState = 0;
                ENABLE_FLIGHT_MODE(ANGLE_MODE);

                return;

            }
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
            currentControlRateProfile->rates[FD_ROLL] = 75;
            currentControlRateProfile->rates[FD_PITCH] = 75;
            DEACTIVATE_RC_MODE(BOXBARO); //deactivate baromode to ensure greater vertical velocity than when the mode is on.

            if (getEstVelocity() < desiredVelocity) {
                  rcData[THROTTLE] = 2000;
            } else
            {
                DISABLE_FLIGHT_MODE(ANGLE_MODE);
                if (flipDirection == FLIP_BACK) {
                    currentControlRateProfile->rates[FD_PITCH] = 110;
                    rcData[PITCH] = 1000;
                    flipState = 2;
                    stateon = millis();
                } else if (flipDirection == FLIP_FRONT) {
                    currentControlRateProfile->rates[FD_PITCH] = 110;
                    rcData[PITCH] = 2000;
                    flipState = 5;
                    stateon = millis();
                } else if (flipDirection == FLIP_LEFT) {
                    currentControlRateProfile->rates[FD_ROLL] = 110;
                    rcData[ROLL] = 1000;
                    flipState = 2;
                    stateon = millis();
                } else {
                    currentControlRateProfile->rates[FD_ROLL] = 110;
                    rcData[ROLL] = 2000;
                    flipState = 5;
                    stateon = millis();
                }
            }
            break;

        case PITCHING:                   //State 2 continue fliping of the drone


//
//            LED_L_OFF;
//            LED_M_ON;
//            LED_R_OFF;


            DEACTIVATE_RC_MODE(BOXBARO);
            DISABLE_FLIGHT_MODE(ANGLE_MODE);

            if (flipDirection == FLIP_BACK) {
                rcData[PITCH] = 1000;
            } else {
                rcData[ROLL] = 1000;
            }

            if (flipDirection == FLIP_BACK && (inclination.values.pitchDeciDegrees > 450)) {
                flipState = 3;
                stateon = millis();
            } else if (flipDirection == FLIP_LEFT
                    && (inclination.values.rollDeciDegrees > 450)) {
                flipState = 3;
                stateon = millis();
            } else {
                flipState = 2;
                if ((millis() - stateon) >= 2000) {
                    flipState = 0;
                    stateon = 0;
                    mwDisarm();
                }
            }
            break;

        case SLOWDOWNANDEXIT: //state 3 slowdown down when the angle of the drone is >225 degrees


//
//            LED_L_OFF;
//            LED_M_OFF;
//            LED_R_ON;
//

            if (flipDirection == FLIP_BACK
                    && inclination.values.pitchDeciDegrees > 450) {
                DEACTIVATE_RC_MODE(BOXBARO);
                DISABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[PITCH] = 1150;
                //rcData[THROTTLE] = 2000;
                if ((millis() - stateon) >= 2000) {
                    flipState = 0;
                    stateon = 0;
                    mwDisarm();
                }
            } else if (flipDirection == FLIP_LEFT
                    && inclination.values.rollDeciDegrees > 450) {
                DEACTIVATE_RC_MODE(BOXBARO);
                DISABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[ROLL] = 1048;
                if ((millis() - stateon) >= 2000) {
                    flipState = 0;
                    stateon = 0;
                }
            } else if (flipDirection == FLIP_LEFT
                    && (inclination.values.rollDeciDegrees >= 0
                            && inclination.values.rollDeciDegrees <= 450
                            && inclination.values.pitchDeciDegrees >= -250
                            && inclination.values.pitchDeciDegrees <= 250)) {
                ACTIVATE_RC_MODE(BOXBARO);
                ENABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[ROLL] = 1048;
                stateon = millis();
                flipState = 4;
            } else if (flipDirection ==FLIP_BACK
                    && (inclination.values.pitchDeciDegrees >= 0
                            && inclination.values.pitchDeciDegrees <= 450
                            && inclination.values.rollDeciDegrees >= -250
                            && inclination.values.rollDeciDegrees <= 250)) {
                ACTIVATE_RC_MODE(BOXBARO);
                ENABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[PITCH] = 1025;
                stateon = millis();
                flipState = 4;
            }
            break;

        case HOLD: // State 4


//
//            LED_L_ON;
//            LED_M_ON;
//            LED_R_ON;

            ACTIVATE_RC_MODE(BOXBARO);
            ENABLE_FLIGHT_MODE(ANGLE_MODE);

            if (flipDirection == FLIP_BACK) {
                rcData[THROTTLE] = 2000;
            } else {
                rcData[THROTTLE] = 2000;
            }


            //Althold.setRelativeAltholdHeight(50);

            currentControlRateProfile->rates[FD_PITCH] = 75;
            currentControlRateProfile->rates[FD_ROLL] = 75;

            if ((millis() - stateon) <= 1500) {
                flipState = 4;
            } else {



//                LED_L_OFF;
//                LED_M_OFF;
//                LED_R_OFF;

              //  Althold.setRelativeAltholdHeight(-150);
                flipState = 0;
                stateon = 0;
            }
            break;

        case PITCHINGPOS: //State 5 continue fliping of the drone

            DISABLE_FLIGHT_MODE(ANGLE_MODE);

            if (flipDirection == FLIP_FRONT) {
                rcData[PITCH] = 2000;
            } else {
                rcData[ROLL] = 1975;
            }

            if (flipDirection == FLIP_FRONT && inclination.values.pitchDeciDegrees < -450) {
                flipState = 6;
                stateon = millis();
            } else if (flipDirection == FLIP_RIGHT && inclination.values.rollDeciDegrees < -450) {
                flipState = 6;
                stateon = millis();
            } else {
                flipState = 5;
                if ((millis() - stateon) >= 2000) {
                    flipState = 0;
                    stateon = 0;
                }
            }
            break;

        case SLOWDOWNANDEXITPOS: //State 6 slowdown down when the angle of the drone is >270 degrees

            if (flipDirection == FLIP_FRONT && inclination.values.pitchDeciDegrees < -450) {
                DEACTIVATE_RC_MODE(BOXBARO);
                DISABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[PITCH] = 1850;

                if ((millis() - stateon) >= 2000) {
                    flipState = 0;
                    stateon = 0;
                }

            } else if (flipDirection == FLIP_RIGHT
                    && inclination.values.rollDeciDegrees < -450) {
                DEACTIVATE_RC_MODE(BOXBARO);
                DISABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[ROLL] = 1975;

                if ((millis() - stateon) >= 2000) {
                    flipState = 0;
                    stateon = 0;
                }
            }else if (flipDirection == FLIP_RIGHT && (inclination.values.rollDeciDegrees >= -450
                            && inclination.values.rollDeciDegrees <= -50
                            && inclination.values.pitchDeciDegrees >= -250
                            && inclination.values.pitchDeciDegrees <= 250)) {
                ACTIVATE_RC_MODE(BOXBARO);
                ENABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[ROLL] = 1850;
                stateon = millis();
                flipState = 7;
            } else if (flipDirection ==FLIP_FRONT && (inclination.values.pitchDeciDegrees >= -450
                            && inclination.values.pitchDeciDegrees <= -50
                            && inclination.values.rollDeciDegrees >= -250
                            && inclination.values.rollDeciDegrees <= 250)) {
                ACTIVATE_RC_MODE(BOXBARO);
                ENABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[PITCH] = 1975;
                stateon = millis();
                flipState = 7;
            }
            break;

        case HOLDPOS: //State 7

            ACTIVATE_RC_MODE(BOXBARO);
            ENABLE_FLIGHT_MODE(ANGLE_MODE);

            if (flipDirection == FLIP_FRONT) {
                rcData[THROTTLE] = 2000;
            } else {
                rcData[ROLL] = 1400;
                rcData[THROTTLE] = 2000;

            }
            currentControlRateProfile->rates[FD_ROLL] = 75;
            currentControlRateProfile->rates[FD_PITCH] = 75;

            if ((millis() - stateon) <= 1500) {
                flipState = 7;
            } else {

            //    Althold.setRelativeAltholdHeight(-150);
                flipState = 0;
                stateon = 0;
               // flipState = 0;
            }
            break;
        }
    } else {



        flipState = 0;
        stateon = 0;
        currentControlRateProfile->rates[FD_PITCH] = 75;
        currentControlRateProfile->rates[FD_ROLL] = 75;
    }
}

void robustChuck()
{
//
//    if (isChukedArmed) {
//        pitch = Flight.get(AG_PITCH);
//        roll = Flight.get(AG_ROLL);
//
//        if (ARMING_FLAG(ARMED) && !isPitchStabelised) {
//
//            if (pitch < 120 && pitch > -120 && roll < 120 && roll > -120) {
//                Control.setRC(RC_PITCH, 1500);
//                isPitchStabelised = true;
//           } else{
//                Control.setRC(RC_PITCH, 2000);
//            }
//
//        }
//
//        if (ARMING_FLAG(ARMED) && !isRollStabelised) {
//
//           if (roll < 120 && roll > -120) {
//                Control.setRC(RC_ROLL, 1500);
//                isRollStabelised = true;
//            } else if (roll > 120 && roll < 900) {
//                Control.setRC(RC_ROLL, 2000);
//            }
//
//        }

//        if (isPitchStabelised && isRollStabelised) {
//               Control.setFailsafeState(CRASH, true);
//        } else {
//            Control.setFailsafeState(CRASH, false);
//          }
 //   }

}

