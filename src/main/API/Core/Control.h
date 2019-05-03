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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum rc_channels {

    RC_ROLL = 0,
    RC_PITCH,
    RC_YAW,
    RC_THROTTLE,
    RC_USER1,
    RC_USER2,
    RC_USER3

} rc_channels_e;

typedef enum f_status {

    FS_Accel_Gyro_Calibration = 0,
    FS_Mag_Calibration,
    FS_Low_battery,
    FS_LowBattery_inFlight,
    FS_Crash,
    FS_Signal_loss,
    FS_Not_ok_to_arm,
    FS_Ok_to_arm,
    FS_Armed

} f_status_e;

typedef enum failsafe {

    LOW_BATTERY = 0, INFLIGHT_LOW_BATTERY, CRASH, ALL

} failsafe_e;

typedef enum {

    COMMAND_TAKE_OFF = 1,
    COMMAND_LAND,

} flight_command_e;

class Control_P {

public:

    bool isOkToArm();
    bool isArmed();
    bool arm(void);
    bool disArm(void);
    void setRcCommand(rc_channels_e channel, int16_t value);
    int16_t getRcData(rc_channels_e channel);
    void disableFlightStatus(bool disable);
    bool checkFlightStatus(f_status_e status);
    f_status_e getCurrentFlightStatus(void);
    void setCommand(flight_command_e command);
    void setFailsafeState(failsafe_e failsafe, bool active);
    void setHeading(int16_t heading);
    void setUserLoopFrequency(float userLoopFrequency);
    void enableDeveloperMode(void);
    void disableDeveloperMode(void);

};

extern Control_P Control;

#ifdef __cplusplus
}
#endif

