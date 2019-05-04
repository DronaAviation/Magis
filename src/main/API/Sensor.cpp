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
#include <stdlib.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/ranging_vl53l0x.h"

#include "sensors/sensors.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "Sensor.h"

int16_t Accelerometer_P::get(axis_e AXIS)
{

    // unit : cm/sec2

    switch (AXIS) {

    case X:

        return (int16_t)(accSmooth[0] * accVelScale);

        break;

    case Y:

        return (int16_t)(accSmooth[1] * accVelScale);
        break;

    case Z:

        return (int16_t)(accSmooth[2] * accVelScale);

        break;

    }

}

int32_t Accelerometer_P::getNetAcc(void)
{

    return netAccMagnitude;
}

int16_t Gyroscope_P::get(axis_e AXIS)
{
    //unit: Decidegrees/sec

    switch (AXIS) {

    case X:

        return (int16_t)(gyroADC[0] / 1.64f);

        break;

    case Y:

        return (int16_t)(gyroADC[1] / 1.64f);

        break;

    case Z:

        return (int16_t)(gyroADC[2] / 1.64f);

        break;

    }
}

int16_t Magnetometer_P::get(axis_e AXIS)
{
    //unit: microTesla

    switch (AXIS) {

    case X:

        return (int16_t)(magADC[0] / 6.82f);

        break;

    case Y:

        return (int16_t)(magADC[1] / 6.82f);

        break;

    case Z:

        return (int16_t)(magADC[2] / 6.82f);

        break;

    }
}

int32_t Barometer_P::get(baro_state_e STATE)
{

    switch (STATE) {

    case PRESSURE:

        return getBaroPressure();  //unit: 100*millibar

        break;

    case TEMPERATURE:

        return getBaroTemperature(); //unit" 100*degreeCelsius

        break;

    }

}

Accelerometer_P Acceleration;
Gyroscope_P Gyroscope;
Magnetometer_P Magnetometer;
Barometer_P Barometer;

