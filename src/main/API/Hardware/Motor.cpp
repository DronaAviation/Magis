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

#include "Motor.h"

#include <stdint.h>

#include "../API-Utils.h"
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

#include "mw.h"

#include "API/API-Utils.h"

void Motor_P::set(motor_e motor, int16_t value)
{

    value = constrain(value, 1000, 2000);

    switch (motor) {
    case M1:

        motor_disarmed[7] = value;
        break;

    case M2:

        motor_disarmed[6] = value;
        break;

    case M3:

        motor_disarmed[4] = value;
        break;

    case M4:

        motor_disarmed[5] = value;
        break;

    case M5:

        motor_disarmed[3] = value;
        break;

    case M6:

        motor_disarmed[2] = value;
        break;

    case M7:

        motor_disarmed[0] = value;
        break;

    case M8:

        motor_disarmed[1] = value;
        break;

    default:
        break;

    }

}

void Motor_P::setDirection(motor_e motor, motor_direction_e direction)
{

    switch (motor) {
    case M1:

        setM1GPIO((bool) direction);
        break;

    case M2:

        setM2GPIO((bool) direction);
        break;

    case M3:

        setM3GPIO((bool) direction);
        break;

    case M4:

        setM4GPIO((bool) direction);
        break;

    }

}

void Motor_P::reverseMotorInit()
{

    reverseMode = true;
}

void Motor_P::setReferenceFrame(reference_frame_e frame)
{

    reverseReferenceFrame = frame;

}

void Motor_P::disableMixerOutput()
{

    motorMixer = false;

}
void Motor_P::enableMixerOutput()
{

    motorMixer = true;

}

Motor_P Motor;
