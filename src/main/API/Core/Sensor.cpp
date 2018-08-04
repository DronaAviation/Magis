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
#include "sensor.h"

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

int16_t Accelerometer_P::getX(void)
{

    return (int16_t)(accSmooth[0] * accVelScale);	// unit : cm/sec2
}

int16_t Accelerometer_P::getY(void)
{

    return (int16_t)(accSmooth[1] * accVelScale);
}

int16_t Accelerometer_P::getZ(void)
{

    return (int16_t)(accSmooth[2] * accVelScale);
}

int32_t Accelerometer_P::getNetAcc(void)
{

    return netAccMagnitude;
}

int16_t Gyroscope_P::getX(void)
{	//unit: degrees/sec

    return (int16_t)(gyroADC[0] / 16.4f);
}

int16_t Gyroscope_P::getY(void)
{

    return (int16_t)(gyroADC[1] / 16.4f);
}

int16_t Gyroscope_P::getZ(void)
{

    return (int16_t)(gyroADC[2] / 16.4f);
}

int16_t Magnetometer_P::getX(void)
{

    return magADC[0];
}

int16_t Magnetometer_P::getY(void)
{

    return magADC[1];
}

int16_t Magnetometer_P::getZ(void)
{

    return magADC[2];
}

int32_t Barometer_P::getPressure(void)
{

    return getBaroPressure();
}

int32_t Barometer_P::getTemperature(void)
{

    return getBaroTemperature();
}

