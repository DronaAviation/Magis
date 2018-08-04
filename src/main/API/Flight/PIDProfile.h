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

class PID_P {

public:

    uint8_t getRoll_P();

    uint8_t getRoll_I();

    uint8_t getRoll_D();

    uint8_t getPitch_P();

    uint8_t getPitch_I();

    uint8_t getPitch_D();

    uint8_t getYaw_P();

    uint8_t getYaw_I();

    uint8_t getYaw_D();

    uint8_t getAlt_P();

    uint8_t getAlt_I();

    uint8_t getAlt_D();

    void setRoll_P(uint8_t roll_P);

    void setRoll_I(uint8_t roll_I);

    void setRoll_D(uint8_t roll_D);

    void setPitch_P(uint8_t pitch_P);

    void setPitch_I(uint8_t pitch_I);

    void setPitch_D(uint8_t pitch_D);

    void setYaw_P(uint8_t yaw_P);

    void setYaw_I(uint8_t yaw_I);

    void setYaw_D(uint8_t yaw_D);

    void setAlt_P(uint8_t alt_P);

    void setAlt_I(uint8_t alt_I);

    void setAlt_D(uint8_t alt_D);

    void setDefault(void);

};

extern PID_P PID;

#ifdef __cplusplus
}
#endif
