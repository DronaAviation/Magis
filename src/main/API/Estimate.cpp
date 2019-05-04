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
#include "flight/posEstimate.h"
#include "config/config.h"
#include "config/runtime_config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "Estimate.h"
#include "API-Utils.h"

int16_t Angle_P::get(angle_e ANGLE)
{

    switch (ANGLE) {

    case AG_ROLL:

        return inclination.values.rollDeciDegrees; // unit : deciDegree

        break;

    case AG_PITCH:

        return inclination.values.pitchDeciDegrees; // unit : deciDegree

        break;

    case AG_YAW:

        return heading; // unit : degree

        break;

    }

}

int16_t Rate_P::get(axis_e AXIS)
{
    // unit deciDegree/sec

    switch (AXIS) {

    case X:

        return currentControlRateProfile->rates[FD_ROLL];

        break;

    case Y:

        return currentControlRateProfile->rates[FD_PITCH];

        break;

    case Z:

        return currentControlRateProfile->rates[FD_YAW];

        break;

    }
}

int16_t Position_P::get(axis_e AXIS)
{
    // unit cm

    switch (AXIS) {

    case X:

        return PositionX;

        break;

    case Y:

        return PositionY;

        break;

    case Z:

        return getEstAltitude();

        break;

    }
}

int16_t Velocity_P::get(axis_e AXIS)
{
    // unit cm/sec

    switch (AXIS) {

    case X:

        return VelocityX;

        break;

    case Y:

        return VelocityY;

        break;

    case Z:

        return getEstVelocity();

        break;

    }
}

Angle_P Angle;
Rate_P Rate;
Position_P Position;
Velocity_P Velocity;

