/*
 * This file is part of Magis.
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
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once



#ifdef __cplusplus
extern "C" {
#endif


extern uint8_t  surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
extern float flowRate[2];          // optical flow angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
extern float bodyRate[2];          // body inertial angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
extern uint32_t last_opticflow_update_ms;

extern float bodyRate1[2];
extern float bodyRate2[2];



void initOpticFlow();
void updateOpticFlow();
void updateSpiOpticFlow();

extern uint8_t opticFlowAddress;

extern uint8_t debugOf;

#ifdef __cplusplus
}
#endif



