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



typedef enum user_pid_gains {

    Kp = 0,
    Ki,
    Kd

} user_pid_gains_e;



class App_P {


public:

    int16_t getAppHeading(void);
    bool isArmSwitchOn(void);
    void setUserPID(user_pid_gains_e gain, uint8_t value);
    uint8_t getUserPID(user_pid_gains_e gain);



};

extern App_P App;

#ifdef __cplusplus
}
#endif
