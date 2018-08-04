/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef __cplusplus
 extern "C" {
#endif 

#ifdef INVERTER
#define INVERTER_OFF digitalLo(INVERTER_GPIO, INVERTER_PIN)
#define INVERTER_ON digitalHi(INVERTER_GPIO, INVERTER_PIN)
#else
#define INVERTER_OFF do {} while(0)
#define INVERTER_ON do {} while(0)
#endif

void initInverter(void);


#ifdef __cplusplus
 }
#endif  

