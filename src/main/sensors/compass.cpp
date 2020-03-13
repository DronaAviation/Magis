/*
 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
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

#include "drivers/sensor.h"
#include "drivers/compass.h"
#include "drivers/compass_hmc5883l.h"
#include "drivers/compass_ak8963.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"

#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

mag_t mag;                   // mag access functions
int16_t mag_declination = 0;

float magneticDeclination = 0.0f;

extern uint32_t currentTime; // FIXME dependency on global variable, pass it in instead.

int16_t magADC[XYZ_AXIS_COUNT];
uint32_t compassLastUpdatedAt=0;
bool have_initial_yaw=false;

sensor_align_e magAlign = (sensor_align_e) 0;
#ifdef MAG
static uint8_t magInit = 0;

void compassInit(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)

    mag.init();
    magInit = 1;
}

#define COMPASS_UPDATE_FREQUENCY_10HZ (1000 * 101.5)
void updateCompass(flightDynamicsTrims_t *magZero, flightDynamicsTrims_t *magScale)
{
    static uint32_t nextUpdateAt, tCal = 0;
    static flightDynamicsTrims_t magZeroTempMin;
    static flightDynamicsTrims_t magZeroTempMax;
    static flightDynamicsTrims_t magScaleTemp;
    uint32_t axis;
    if ((int32_t) (currentTime - nextUpdateAt) < 0)
        return;
    nextUpdateAt = currentTime + COMPASS_UPDATE_FREQUENCY_10HZ;
    compassLastUpdatedAt=currentTime;
    mag.read(magADC);
    alignSensors(magADC, magADC, magAlign);
    if (STATE(CALIBRATE_MAG)) {
        tCal = nextUpdateAt;
        have_initial_yaw=false;
        for (axis = 0; axis < 3; axis++) {
            magZero->raw[axis] = 0;
            magZeroTempMin.raw[axis] = magADC[axis];
            magZeroTempMax.raw[axis] = magADC[axis];
        }
        DISABLE_STATE(CALIBRATE_MAG);
    }
    if (tCal != 0) {
        if ((nextUpdateAt - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            //LED0_TOGGLE; //drona
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin.raw[axis])
                    magZeroTempMin.raw[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax.raw[axis])
                    magZeroTempMax.raw[axis] = magADC[axis];
            }

        } else {
            tCal = 0;
            for (axis = 0; axis < 3; axis++) {
                magZero->raw[axis] = (magZeroTempMin.raw[axis] + magZeroTempMax.raw[axis]) / 2; // Calculate offsets: hard iron correction
                magScaleTemp.raw[axis] = ((magZeroTempMax.raw[axis] - magZeroTempMin.raw[axis]) * 10) / 2;
            }
            for (axis = 0; axis < 3; axis++) {
                magScale->raw[axis] = ((magScaleTemp.raw[0] + magScaleTemp.raw[1] + magScaleTemp.raw[2]) * 10) / (3 * magScaleTemp.raw[axis]);
            }
            saveConfigAndNotify();
        }
    }

    if (magInit) {              // we apply offset only once mag calibration is done
        magADC[X] -= magZero->raw[X];
        magADC[Y] -= magZero->raw[Y];
        magADC[Z] -= magZero->raw[Z];

        magADC[X] *= magScale->raw[X];
        magADC[Y] *= magScale->raw[Y];
        magADC[Z] *= magScale->raw[Z];

        magADC[X] /= 10;
        magADC[Y] /= 10;
        magADC[Z] /= 10;
    }
}
#endif

void recalculateMagneticDeclination(void)
{
    int16_t deg, min;

    if (sensors(SENSOR_MAG)) {
        // calculate magnetic declination
        deg = mag_declination / 100;
        min = mag_declination % 100;

        magneticDeclination = (deg + ((float) min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
    } else {
        magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
    }

}
