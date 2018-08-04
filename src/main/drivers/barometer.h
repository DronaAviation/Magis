/*
 * This file is part of Cleanflight and Magis.
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

typedef void (*baroOpStartFuncPtr)(void);                       // baro start operation
typedef uint32_t (*baroOpGetFuncPtr)(void);                       // baro start operation
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature, uint32_t P, uint32_t T); // baro calculation (filled params are pressure and temperature)

typedef struct baro_s {
        uint16_t ut_delay;
        uint16_t up_delay;
        baroOpStartFuncPtr start_ut;
        baroOpGetFuncPtr get_ut;
        baroOpStartFuncPtr start_up;
        baroOpGetFuncPtr get_up;
        baroCalculateFuncPtr calculate;
} baro_t;

#ifdef __cplusplus
}
#endif 
