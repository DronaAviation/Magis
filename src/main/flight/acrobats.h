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

#ifdef __cplusplus
extern "C" {
#endif

#define ASCEND 1
#define PITCHING 2
#define SLOWDOWNANDEXIT 3
#define HOLD 4
#define PITCHINGPOS 5
#define SLOWDOWNANDEXITPOS 6
#define HOLDPOS 7

typedef enum {

    FLIP_BACK=0,
    FLIP_FRONT,
    FLIP_RIGHT,
    FLIP_LEFT,
    FLIP_COUNT

} flipDirection_e;


extern uint8_t flipDirection;
extern uint32_t flipState;
extern uint32_t flipStartTime;

extern bool isPitchStabelised;
extern bool isRollStabelised;

void flip(bool doFlip);
void robustChuck();

#ifdef __cplusplus
}
#endif
