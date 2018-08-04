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

#define TOTAL_CMDS 25

typedef enum {
    C_TAKE_OFF,
    C_LAND,
    C_SLEEP,
    C_ARM_MOTORS,
    C_GO_TO_WAY_POINT,
    C_DISARM_MOTORS,
    C_LOOP_FROM_START
} cmd_type;

typedef struct command {
    cmd_type id;
    int16_t pos_x;
    int16_t pos_y;
    int16_t height;
} cmd_def;

void command_add(cmd_def command);

bool command_verify(void);

void command_next(void);

void command_previous(void);

void command_run(uint32_t current_time);

void command_jump(int16_t jump);

extern int32_t debugLCvar;
