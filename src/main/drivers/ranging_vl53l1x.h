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

#include "vl53l1_def.h"
#include "vl53l1_LL_device.h"
#include "vl53l1_platform.h"
#include "vl53l1_platform_user_data.h"
#include "API/Specifiers.h"

class LaserSensor_L1 {

    int16_t range;
    VL53L1_Dev_t MyDevice_L1;
    unibus_e statusLEDPin;

public:

    void init();
    void setAddress(uint8_t address);
    int16_t startRanging();
    int16_t getLaserRange();

};



void ranging_init_L1(void);
void getRange_L1(void);
bool isTofDataNew_L1(void);
bool isOutofRange_L1(void);



extern VL53L1_Error Global_Status_L1;
extern VL53L1_RangingMeasurementData_t RangingMeasurementData_L1;

extern uint8_t Range_Status_L1;
extern uint16_t NewSensorRange_L1;
extern uint16_t debug_range_L1;
extern bool startRanging_L1;
extern bool isTofDataNewflag_L1;
extern bool useRangingSensor_L1;


#ifdef __cplusplus
}
#endif
