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
#include "common/axis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UWB,
    WHYCON,
    VICON

} localisation_type;

/*
 typedef enum {
 LC_TAKE_OFF,
 LC_LAND,
 LC_SLEEP,
 LC_ARM_MOTORS,
 LC_GO_TO_WAY_POINT,
 LC_DISARM_MOTORS,
 LC_LOOP_FROM_START
 } localisation_cmd_type;
 */

class Localisation_P {

public:

    void init(localisation_type localisation);

    int16_t get(axis_e AXIS);

    /*

     void setCommand(localisation_cmd_type command);

     void setCommand(localisation_cmd_type command, int16_t desiredX, int16_t desiredY);

     void setCommand(localisation_cmd_type command, int16_t desiredX, int16_t desiredY, int16_t desiredZ);

     void startLocalisation();

     void stopLocalisation();

     */

};

extern Localisation_P Localisation;

#ifdef __cplusplus
}
#endif
