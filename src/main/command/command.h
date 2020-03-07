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

#include "../API/Utils.h"

typedef enum {

    NONE = 0,
    TAKE_OFF,
    LAND,
    B_FLIP,
    F_FLIP,
    R_FLIP,
    L_FLIP

} command_e;

typedef enum {

    FINISHED = 0, RUNNING, ABORT

} command_status_e;

extern uint8_t current_command;

extern uint8_t command_status;

extern bool isLanding;

extern bool setTakeOffAlt;

extern bool setTakeOffThrottle;

extern bool setTakeOffTimer;

extern bool setLandTimer;

extern uint32_t takeOffLoopTime;


extern int32_t takeOffThrottle;

extern bool isTookOff;

extern bool isTakeOffHeightSet;


extern uint16_t takeOffHeight;
extern uint16_t landThrottle;
extern bool isUserLandCommand;


void resetCommandRCflag(void);

void executeCommand();

void updateCommandStatus();


