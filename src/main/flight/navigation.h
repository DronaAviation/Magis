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

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/atomic.h"
#include "common/maths.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"


#include "rx/rx.h"

#include "io/serial.h"

#ifdef __cplusplus
extern "C" {
#endif 

// navigation mode
typedef enum {
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_WP
} navigationMode_e;

// FIXME ap_mode is badly named, it's a value that is compared to rcCommand, not a flag at it's name implies.

typedef struct gpsProfile_s {
        uint16_t gps_wp_radius; // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
        uint8_t gps_lpf;                      // Low pass filter cut frequency for derivative calculation (default 20Hz)
        uint8_t nav_slew_rate;                  // Adds a rate control to nav output, will smoothen out nav angle spikes
        uint8_t nav_controls_heading;        // copter faces toward the navigation point, maghold must be enabled for it
        uint16_t nav_speed_min;                 // cm/sec
        uint16_t nav_speed_max;                 // cm/sec
        uint16_t ap_mode; // Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks, creating a deadspan for GPS
} gpsProfile_t;

extern int16_t GPS_angle[ANGLE_INDEX_COUNT];                // it's the angles that must be applied for GPS correction

extern int32_t GPS_home[2];
extern int32_t GPS_hold[2];

extern uint16_t GPS_distanceToHome;        // distance to home point in meters
extern int16_t GPS_directionToHome;        // direction to home or hol point in degrees

extern navigationMode_e nav_mode;          // Navigation mode

extern int32_t posRateIntegralLat;

extern int32_t posRateIntegralLon;

extern uint32_t latError;

extern uint32_t lonError;


void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void gpsUseProfile(gpsProfile_t *gpsProfileToUse);
void gpsUsePIDs(pidProfile_t *pidProfile);
void updateGpsStateForHomeAndHoldMode(void);
void updateGpsWaypointsAndMode(void);
void navigationInit(gpsProfile_t *initialGpsProfile, pidProfile_t *pidProfile);
void onGpsNewData(void);

#ifdef __cplusplus
}
#endif 
