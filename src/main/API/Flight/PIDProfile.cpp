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

#include <stdint.h>

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/light_led.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"

#include "telemetry/telemetry.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"
#include "flight/acrobats.h"

#include "mw.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "PIDProfile.h"

uint8_t PID_P::getRoll_P()
{

    return currentProfile->pidProfile.P8[ROLL];

}

uint8_t PID_P::getRoll_I()
{

    return currentProfile->pidProfile.I8[ROLL];

}

uint8_t PID_P::getRoll_D()
{

    return currentProfile->pidProfile.D8[ROLL];
}

uint8_t PID_P::getPitch_P()
{

    return currentProfile->pidProfile.P8[PITCH];

}

uint8_t PID_P::getPitch_I()
{

    return currentProfile->pidProfile.I8[PITCH];

}

uint8_t PID_P::getPitch_D()
{

    return currentProfile->pidProfile.D8[PITCH];

}

uint8_t PID_P::getYaw_P()
{

    return currentProfile->pidProfile.P8[YAW];

}

uint8_t PID_P::getYaw_I()
{

    return currentProfile->pidProfile.I8[YAW];

}

uint8_t PID_P::getYaw_D()
{

    return currentProfile->pidProfile.D8[YAW];

}

uint8_t PID_P::getAlt_P()
{

    return currentProfile->pidProfile.P8[PIDALT];

}

uint8_t PID_P::getAlt_I()
{

    return currentProfile->pidProfile.I8[PIDALT];

}

uint8_t PID_P::getAlt_D()
{

    return currentProfile->pidProfile.D8[PIDALT];

}

void PID_P::setRoll_P(uint8_t roll_P)
{

    currentProfile->pidProfile.P8[ROLL] = roll_P;

}

void PID_P::setRoll_I(uint8_t roll_I)
{

    currentProfile->pidProfile.I8[ROLL] = roll_I;

}

void PID_P::setRoll_D(uint8_t roll_D)
{

    currentProfile->pidProfile.D8[ROLL] = roll_D;

}

void PID_P::setPitch_P(uint8_t pitch_P)
{

    currentProfile->pidProfile.P8[PITCH] = pitch_P;

}

void PID_P::setPitch_I(uint8_t pitch_I)
{

    currentProfile->pidProfile.I8[PITCH] = pitch_I;

}

void PID_P::setPitch_D(uint8_t pitch_D)
{

    currentProfile->pidProfile.D8[PITCH] = pitch_D;

}

void PID_P::setYaw_P(uint8_t yaw_P)
{

    currentProfile->pidProfile.P8[YAW] = yaw_P;

}

void PID_P::setYaw_I(uint8_t yaw_I)
{

    currentProfile->pidProfile.I8[YAW] = yaw_I;

}

void PID_P::setYaw_D(uint8_t yaw_D)
{

    currentProfile->pidProfile.D8[YAW] = yaw_D;

}

void PID_P::setAlt_P(uint8_t alt_P)
{

    currentProfile->pidProfile.P8[PIDALT] = alt_P;

}

void PID_P::setAlt_I(uint8_t alt_I)
{

    currentProfile->pidProfile.I8[PIDALT] = alt_I;

}

void PID_P::setAlt_D(uint8_t alt_D)
{

    currentProfile->pidProfile.D8[PIDALT] = alt_D;

}

void PID_P::setDefault(void)
{

    currentProfile->pidProfile.P8[ROLL] = 40;
    currentProfile->pidProfile.I8[ROLL] = 10;
    currentProfile->pidProfile.D8[ROLL] = 30;
    currentProfile->pidProfile.P8[PITCH] = 40;
    currentProfile->pidProfile.I8[PITCH] = 10;
    currentProfile->pidProfile.D8[PITCH] = 30;
    currentProfile->pidProfile.P8[YAW] = 80;
    currentProfile->pidProfile.I8[YAW] = 70;
    currentProfile->pidProfile.D8[YAW] = 5;
    currentProfile->pidProfile.P8[PIDALT] = 100;
    currentProfile->pidProfile.I8[PIDALT] = 0;
    currentProfile->pidProfile.D8[PIDALT] = 30;

}

PID_P PID;
