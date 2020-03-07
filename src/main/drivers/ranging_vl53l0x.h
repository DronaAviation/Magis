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

#include "vl53l0x_def.h"
#include "vl53l0x_device.h"
#include "vl53l0x_platform.h"
#include "API/Specifiers.h"

class LaserSensor {

    int16_t range;
    VL53L0X_Dev_t MyDevice;
    unibus_e statusLEDPin;

public:

    void init();
    void setAddress(uint8_t address);
    int16_t startRanging();
    int16_t getLaserRange();

};



void ranging_init(void);
void getRange(void);
bool isTofDataNew(void);
bool isOutofRange(void);



extern VL53L0X_Error Global_Status;
extern VL53L0X_RangingMeasurementData_t RangingMeasurementData;

extern uint8_t Range_Status;
extern uint16_t NewSensorRange;
extern uint16_t debug_range;
extern bool startRanging;
extern bool isTofDataNewflag;
extern bool useRangingSensor;


#ifdef __cplusplus
}
#endif
