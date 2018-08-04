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
#include "Althold.h"

int32_t Althold_P::getEstimatedAltitude(void)
{
    return altitudeHoldGetEstimatedAltitude();

}

int32_t Althold_P::getVelocityZ(void)
{

    return getSetVelocity();
}

void Althold_P::activateAlthold(bool activate)
{

    if (activate) {

        AUX3_VALUE = 1500;
    } else {
        AUX3_VALUE = 1200;

    }

}

bool Althold_P::isAltholdModeActive(void)
{

    return IS_RC_MODE_ACTIVE(BOXARM);
}

void Althold_P::setAltholdHeight(int32_t height)
{

    setUserAltitude(height);

}

void Althold_P::setRelativeAltholdHeight(int32_t height)
{

    setRelativeUserAltitude(height);

}

int32_t Althold_P::getAltholdHeight(void)
{
    return AltHold;

}

Althold_P Althold;

