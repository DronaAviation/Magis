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
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "common/axis.h"
#include "io/rc_controls.h"
#include "flight/pid.h"//PS2

bool PositionController(int16_t desiredX, int16_t desiredY, int16_t desiredZ);
void VelocityController(int16_t VdesiredX, int16_t VdesiredY);
void PosVelController(int16_t desiredX, int16_t desiredY, int16_t desiredVel);
bool SimpleController(int16_t desiredX, int16_t desiredY, int16_t desiredZ);
void selectVelOrPosmode(void);


int16_t getrcDataRoll(void);
int16_t getrcDataPitch(void);
int16_t getDesiredVelocityX();
int16_t getDesiredVelocityY();

void setdesiredHeight(int32_t desiredZ);
int32_t getdesiredHeight(void);
void configurePosHold(pidProfile_t *initialPidProfile,rcControlsConfig_t *rcControlsConfigPointer ); //PS2
void resetPosIntegral(void);
void resetPosController(void);

extern uint8_t HeightAchieved;
extern int16_t PID_x;
extern int16_t PID_y;
extern int16_t VdesiredX;
extern int16_t VdesiredY;

extern uint8_t velocityControlX;
extern uint8_t velocityControlY;

extern int32_t desiredPositionX;
extern int32_t desiredPositionY;



#ifdef __cplusplus
}
#endif

