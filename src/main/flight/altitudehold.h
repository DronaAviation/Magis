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
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef __cplusplus
extern "C" {
#endif 

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "flight/pid.h"

#include "sensors/barometer.h"



extern int16_t accalttemp;
extern int16_t althold_throttle;
extern int16_t max_altitude;
extern int32_t AltHold;
extern int32_t vario;
extern int32_t baroAlt_offset_print;

extern float ToF_Height;

extern barometerConfig_t *barometerConfig_tmp;

void configureAltitudeHold(pidProfile_t *initialPidProfile,barometerConfig_t *intialBarometerConfig, rcControlsConfig_t *initialRcControlsConfig, escAndServoConfig_t *initialEscAndServoConfig);
void applyAltHold(airplaneConfig_t *airplaneConfig);
void updateAltHoldState(void);
void updateSonarAltHoldState(void);

int32_t altitudeHoldGetEstimatedAltitude(void);
int32_t getSetVelocity(void);
void AltRst(void);




void correctedWithTof(float ToF_Height);
void correctedWithBaro(float baroAlt, float dt);

void checkReading();
void checkBaro();

void updateGains();
void updateTimeConstantandGains(uint8_t timeConstant);
float getTimeConstant();
void apmCalculateEstimatedAltitude(uint32_t currtime);
void setUserAltitude(int32_t altitude);
void setRelativeUserAltitude(int32_t altitude);
int32_t getEstAltitude();


bool limitAltitude();


#ifdef __cplusplus
}
#endif
