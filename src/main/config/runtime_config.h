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

#pragma once

// FIXME some of these are flight modes, some of these are general status indicators
#ifdef __cplusplus
extern "C" {
#endif 

typedef enum {
    OK_TO_ARM = (1 << 0),
    PREVENT_ARMING = (1 << 1),
    ARMED = (1 << 2)
} armingFlag_e;

extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))
#define ARMING_FLAG(mask) (armingFlags & (mask))

typedef enum {
    ANGLE_MODE = (1 << 0),
    HORIZON_MODE = (1 << 1),
    MAG_MODE = (1 << 2),
    BARO_MODE = (1 << 3),
    GPS_HOME_MODE = (1 << 4),
    GPS_HOLD_MODE = (1 << 5),
    HEADFREE_MODE = (1 << 6),
    UNUSED_MODE = (1 << 7), // old autotune
    PASSTHRU_MODE = (1 << 8),
    SONAR_MODE = (1 << 9),
    FAILSAFE_MODE = (1 << 10),
    GTUNE_MODE = (1 << 11),
} flightModeFlags_e;

extern uint16_t flightModeFlags;

#define DISABLE_FLIGHT_MODE(mask) disableFlightMode(mask)
#define ENABLE_FLIGHT_MODE(mask) enableFlightMode(mask)
#define FLIGHT_MODE(mask) (flightModeFlags & (mask))

typedef enum {
    GPS_FIX_HOME = (1 << 0),
    GPS_FIX = (1 << 1),
    CALIBRATE_MAG = (1 << 2),
    SMALL_ANGLE = (1 << 3),
    FIXED_WING = (1 << 4),        // set when in flying_wing or airplane mode. currently used by althold selection code
} stateFlags_t;

#define DISABLE_STATE(mask) (stateFlags &= ~(mask))
#define ENABLE_STATE(mask) (stateFlags |= (mask))
#define STATE(mask) (stateFlags & (mask))

extern uint8_t stateFlags;

uint16_t enableFlightMode(flightModeFlags_e mask);
uint16_t disableFlightMode(flightModeFlags_e mask);
bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);

void mwDisarm(void);



typedef enum{
    Accel_Gyro_Calibration = 0,
	Mag_Calibration = 1,
	Armed = 8,
	Ok_to_arm = 7,
	Not_ok_to_arm = 6,
	Signal_loss = 5,
    Crash = 2,
    LowBattery_inFlight = 4,
    Low_battery = 3
}FlightStatus_e;



typedef enum{
    App_Accel_Gyro_Calibration = 0,
    App_Mag_Calibration = 1,
    App_Armed = 2,
    App_Ok_to_arm = 3,
    App_Not_ok_to_arm = 4,
    App_Signal_loss = 5,
    App_Crash = 6,
    App_LowBattery_inFlight = 7,
    App_Low_battery = 8
}AppFlightStatus_e;




#ifdef FLIGHT_STATUS_INDICATOR
void flightStatusIndicator(void);
#endif
extern uint16_t flightIndicatorFlag;



void set_FSI(FlightStatus_e flag);
void reset_FSI(FlightStatus_e flag);
bool status_FSI(FlightStatus_e flag);


#ifdef __cplusplus
}
#endif 
