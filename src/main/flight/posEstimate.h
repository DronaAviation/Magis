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
#include "flight/pid.h"//PS2

#define	LOC_UWB 0
#define	LOC_WHYCON 1
#define	LOC_VICON  2


void addHistPositionBaseEstXY(float positionX, float positionY);
void getFrontHistPositionBaseEstXY(float *posbaseX, float *posbaseY);
bool isPositionBaseXYQueueIsFull(void);

void PosXYEstimate(uint32_t currentTime);
void resetPosition(void);
void updatePosGains(void);
void setPos(float newX, float newY);

void configurePosHold2(pidProfile_t *initialPidProfile);

extern int16_t posX, posY, posZ, deltaTime;
extern int8_t Quality;
extern bool new_position;
extern int8_t localisationType;

extern float PositionX, PositionY;
extern float PrevPositionX, PrevPositionY;
extern int16_t VelocityX, VelocityY;
extern int16_t inputVx, inputVy;
extern float whyconZ;
extern float accel_EF[2], accel_EF_prev[2];



#ifdef __cplusplus
}
#endif
