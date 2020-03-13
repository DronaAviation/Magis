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


extern float debugOpticFlowVar;
extern float debugOpticFlowVar1;
extern float debugOpticFlowVar2;
extern float debugOpticFlowVar3;
extern float debugOpticFlowVar4;
extern float debugOpticFlow[2];
extern float debugOpticFlow1[2];
extern float debugOpticFlow2[3];



extern float sensor_flow_hf[2];
extern float accel_hf[3];
extern float accel_hf_prev[3];
extern float velocity_hf[3];
extern float opticflowHeight;
void runFlowHold(uint32_t currentTime);


#ifdef __cplusplus
}
#endif


