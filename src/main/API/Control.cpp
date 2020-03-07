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
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/ranging_vl53l0x.h"
#include "drivers/opticflow_paw3903.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/filter.h"
#include "flight/acrobats.h"
#include "flight/posEstimate.h"
#include "flight/posControl.h"
#include "flight/opticflow.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "mw.h"

#include "Control.h"
#include "API-Utils.h"

int32_t DesiredAngle_P::get(angle_e ANGLE)
{

    return desiredAngle[ANGLE];

}

void DesiredAngle_P::set(angle_e ANGLE, int32_t angle)
{

    switch (ANGLE) {

    case AG_ROLL:
        //inclination.values.rollDeciDegrees;
        break;

    case AG_PITCH:
        //inclination.values.pitchDeciDegrees;
        break;

    case AG_YAW:
        magHold = angle;
        userHeading = angle;
        isUserHeadingSet = true;
        break;

    default:

        break;
    }

}

int32_t DesiredRate_P::get(angle_e ANGLE)
{

    return desiredRate[ANGLE];

}

void DesiredRate_P::set(angle_e ANGLE, int32_t rate)
{

    switch (ANGLE) {

    case AG_ROLL:

        //   currentControlRateProfile->rates[FD_ROLL]=rate;

        break;

    case AG_PITCH:

        //    currentControlRateProfile->rates[FD_PITCH]=rate;

        break;

    case AG_YAW:

        //   currentControlRateProfile->rates[FD_YAW]=rate;

        break;

    default:

        break;

    }

}

int32_t DesiredPosition_P::get(axis_e AXIS)
{

    switch (AXIS) {

    case X:

        return 0;

        break;

    case Y:

        return 0;

        break;

    case Z:

        return getSetAltitude();

        break;

    }

}

void DesiredPosition_P::set(axis_e AXIS, int32_t position)
{

    switch (AXIS) {

    case X:

        break;

    case Y:

        break;

    case Z:

        setAltitude(position);

        break;

    }

}

void DesiredPosition_P::setRelative(axis_e AXIS, int32_t position)
{

    switch (AXIS) {

    case X:

        break;

    case Y:

        break;

    case Z:

        setRelativeAltitude(position);

        break;

    }

}

int32_t DesiredVelocity_P::get(axis_e AXIS)
{

    switch (AXIS) {

    case X:

        return 0;

        break;

    case Y:

        return 0;

        break;

    case Z:

        return getSetVelocity();

        break;

    }

}

void DesiredVelocity_P::set(axis_e AXIS, int32_t velocity)
{

    switch (AXIS) {

    case X:

        break;

    case Y:

        break;

    case Z:

        //   setRelativeUserAltitude(position);

        break;

    }

}

/*
 typedef enum{

 ROLL_LIMIT,
 PITCH_LIMIT,
 YAW_LIMIT,
 ROLL_RATE_LIMIT,
 PITCH_RARE_LIMIT,
 YAW_RATE_LIMIT,
 POS_X_LIMIT,
 POS_Y_LIMIT,
 POS_Z_LIMIT,
 VEL_x_LIMIT,
 VEL_Y_LIMIT,
 VEL_Z_LIMIT,
 USER_LIMIT

 }limit_profile_e;




 class ControlLimit_P {
 public:

 int16_t get(limit_profile_e LIMIT);

 void set(limit_profile_e LIMIT, int16_t limit);


 };



 int16_t ControlLimit_P::get(limit_profile_e LIMIT)
 {



 }



 void ControlLimit_P::set(limit_profile_e LIMIT, int16_t limit)
 {





 }


 */

void PIDProfile_P::get(pid_profile_e PROFILE, PID* pid)
{

    switch (PROFILE) {

    case PID_ROLL:

        pid->p = currentProfile->pidProfile.P8[ROLL];
        pid->i = currentProfile->pidProfile.I8[ROLL];
        pid->d = currentProfile->pidProfile.D8[ROLL];

        break;

    case PID_PITCH:

        pid->p = currentProfile->pidProfile.P8[PITCH];
        pid->i = currentProfile->pidProfile.I8[PITCH];
        pid->d = currentProfile->pidProfile.D8[PITCH];

        break;

    case PID_YAW:

        pid->p = currentProfile->pidProfile.P8[YAW];
        pid->i = currentProfile->pidProfile.I8[YAW];
        pid->d = currentProfile->pidProfile.D8[YAW];

        break;

    case PID_ALT:

        pid->p = currentProfile->pidProfile.P8[PIDALT];
        pid->i = currentProfile->pidProfile.I8[PIDALT];
        pid->d = currentProfile->pidProfile.D8[PIDALT];

        break;

    case PID_USER:

        pid->p = currentProfile->pidProfile.P8[PIDUSER];
        pid->i = currentProfile->pidProfile.I8[PIDUSER];
        pid->d = currentProfile->pidProfile.D8[PIDUSER];

        break;

    default:

        break;

    }

}

void PIDProfile_P::set(pid_profile_e PROFILE, PID* pid)
{

    switch (PROFILE) {

    case PID_ROLL:

        currentProfile->pidProfile.P8[ROLL] = pid->p;
        currentProfile->pidProfile.I8[ROLL] = pid->i;
        currentProfile->pidProfile.D8[ROLL] = pid->d;

        break;

    case PID_PITCH:

        currentProfile->pidProfile.P8[PITCH] = pid->p;
        currentProfile->pidProfile.I8[PITCH] = pid->i;
        currentProfile->pidProfile.D8[PITCH] = pid->d;

        break;

    case PID_YAW:

        currentProfile->pidProfile.P8[YAW] = pid->p;
        currentProfile->pidProfile.I8[YAW] = pid->i;
        currentProfile->pidProfile.D8[YAW] = pid->d;

        break;

    case PID_ALT:

        currentProfile->pidProfile.P8[PIDALT] = pid->p;
        currentProfile->pidProfile.I8[PIDALT] = pid->i;
        currentProfile->pidProfile.D8[PIDALT] = pid->d;

        break;

    case PID_USER:

        currentProfile->pidProfile.P8[PIDUSER] = pid->p;
        currentProfile->pidProfile.I8[PIDUSER] = pid->i;
        currentProfile->pidProfile.D8[PIDUSER] = pid->d;

        break;

    default:

        break;

    }

}

void PIDProfile_P::setDefault(void)
{

    currentProfile->pidProfile.P8[ROLL] = 40;
    currentProfile->pidProfile.I8[ROLL] = 10;
    currentProfile->pidProfile.D8[ROLL] = 30;
    currentProfile->pidProfile.P8[PITCH] = 40;
    currentProfile->pidProfile.I8[PITCH] = 10;
    currentProfile->pidProfile.D8[PITCH] = 30;
    currentProfile->pidProfile.P8[YAW] = 150;
    currentProfile->pidProfile.I8[YAW] = 70;
    currentProfile->pidProfile.D8[YAW] = 50;
    currentProfile->pidProfile.P8[PIDALT] = 100;
    currentProfile->pidProfile.I8[PIDALT] = 0;
    currentProfile->pidProfile.D8[PIDALT] = 30;
    currentProfile->pidProfile.P8[PIDUSER] = 0;
    currentProfile->pidProfile.I8[PIDUSER] = 0;
    currentProfile->pidProfile.D8[PIDUSER] = 0;

}

void Failsafe_P::enable(failsafe_e FAILSAFE)
{

    switch (FAILSAFE) {

    case LOW_BATTERY:

        fsLowBattery = true;
        break;

    case INFLIGHT_LOW_BATTERY:

        fsInFlightLowBattery = true;
        break;

    case CRASH:
        fsCrash = true;
        break;

    case ALL:
        fsLowBattery = true;
        fsInFlightLowBattery = true;
        fsCrash = true;
        break;

    }

}

void Failsafe_P::disable(failsafe_e FAILSAFE)
{

    switch (FAILSAFE) {

    case LOW_BATTERY:

        fsLowBattery = false;
        break;

    case INFLIGHT_LOW_BATTERY:

        fsInFlightLowBattery = false;
        break;

    case CRASH:
        fsCrash = false;
        break;

    case ALL:
        fsLowBattery = false;
        fsInFlightLowBattery = false;
        fsCrash = false;
        break;

    }

}

DesiredAngle_P DesiredAngle;
DesiredRate_P DesiredRate;
DesiredPosition_P DesiredPosition;
DesiredVelocity_P DesiredVelocity;
PIDProfile_P PIDProfile;
Failsafe_P Failsafe;
