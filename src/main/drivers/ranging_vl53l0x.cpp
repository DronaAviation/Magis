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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

#include "vl53l0x_api_core.h"
#include "vl53l0x_api_strings.h"
#include "vl53l0x_def.h"
#include "vl53l0x_api.h"
#include "vl53l0x_types.h"

#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/bus_i2c.h"
#include "drivers/system.h"
#include "ranging_vl53l0x.h"
#include "API/Peripheral.h"
#include "API/Utils.h"



#define LASER_LPS 0.1


VL53L0X_Dev_t MyDevice;
//VL53L0X_Dev_t *pMyDevice = &MyDevice;

VL53L0X_Error Global_Status = 0;
VL53L0X_RangingMeasurementData_t RangingMeasurementData;


uint8_t Range_Status = 0;
uint16_t NewSensorRange=0;
uint16_t debug_range = 0;
bool isTofDataNewflag = false;
bool out_of_range = false;
bool startRanging = false;
bool useRangingSensor=false;

Interval rangePoll;




void update_status(VL53L0X_Error Status)
{
    Global_Status = Status;
}

#ifdef LASER_TOF

void ranging_init(void)
{
    VL53L0X_Error Status = Global_Status;

    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    MyDevice.I2cDevAddr = 0x29;
    MyDevice.comms_type = 1;
    MyDevice.comms_speed_khz = 400;

    Status = VL53L0X_DataInit(&MyDevice); // Data initialization

    update_status(Status);

    if(Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_StaticInit(&MyDevice); // Device Initialization
        update_status(Status);

    }

    if( Global_Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_PerformRefSpadManagement( &MyDevice, &refSpadCount, &isApertureSpads ); // Device Initialization
        update_status(Status);

    }

    if( Global_Status == VL53L0X_ERROR_NONE ){
        Status = VL53L0X_PerformRefCalibration( &MyDevice, &VhvSettings, &PhaseCal ); // Device Initialization
        update_status(Status);

    }

    if(Global_Status == VL53L0X_ERROR_NONE){
        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        Status = VL53L0X_SetDeviceMode(&MyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);// Setup in single ranging mode
        update_status(Status);

    }

    // Enable/Disable Sigma and Signal check
    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.1*65536));
        update_status(Status);


    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(60*65536));

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&MyDevice,33000);
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
        update_status(Status);
    
    }


}

void getRange()
{
    VL53L0X_Error Status = Global_Status;
    static uint8_t dataFlag = 0, SysRangeStatus = 0;
    static bool startNow = true;


    if(rangePoll.set(33,true)) {

    if(Global_Status == VL53L0X_ERROR_NONE) {
        if(startNow) {
            Status = VL53L0X_StartMeasurement(&MyDevice);
            update_status(Status);
            startNow = false;
        }
    }

//    if(Global_Status == VL53L0X_ERROR_NONE) {
//
//        if(!startNow) {
//            Status = VL53L0X_GetMeasurementDataReady(&MyDevice,&dataFlag);
//            update_status(Status);
//
//        }
//    }

    if(Global_Status == VL53L0X_ERROR_NONE) {
//        if(dataFlag) {

            Status = VL53L0X_GetRangingMeasurementData(&MyDevice, &RangingMeasurementData);
            update_status(Status);
            // printInt("return status#4:",Global_Status);

//            if(RangingMeasurementData.RangeDMaxMilliMeter != 0) {
//                debug_range = RangingMeasurementData.RangeDMaxMilliMeter/10;
//            }

            startNow = true;
            isTofDataNewflag = true;
            Range_Status = RangingMeasurementData.RangeStatus;
            if(RangingMeasurementData.RangeStatus == 0) {

            	if(RangingMeasurementData.RangeMilliMeter<2000){

              //  NewSensorRange = RangingMeasurementData.RangeMilliMeter;

                NewSensorRange = NewSensorRange*(1-LASER_LPS)+RangingMeasurementData.RangeMilliMeter*LASER_LPS;

                out_of_range = false;

            	} else {
                    out_of_range = true;
				}

            } else
            out_of_range = true;
//        }
    }

    }


}

bool isTofDataNew(void)
{
    return isTofDataNewflag;
}

bool isOutofRange(void)
{
    return out_of_range;
}

#endif


void LaserSensor::init()
{



    VL53L0X_Error Status = Global_Status;

    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    this->MyDevice.I2cDevAddr = 0x29;
    this->MyDevice.comms_type = 1;
    this->MyDevice.comms_speed_khz = 400;

   // this->statusLEDPin=pin;

    Status = VL53L0X_DataInit(&this->MyDevice); // Data initialization

    update_status(Status);

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_StaticInit(&this->MyDevice); // Device Initialization
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_PerformRefSpadManagement(&this->MyDevice, &refSpadCount, &isApertureSpads); // Device Initialization
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_PerformRefCalibration(&this->MyDevice, &VhvSettings,&PhaseCal);           // Device Initialization
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        Status = VL53L0X_SetDeviceMode(&this->MyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        update_status(Status);

    }

    // Enable/Disable Sigma and Signal check
    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&this->MyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&this->MyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&this->MyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t) (0.1 * 65536));
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&this->MyDevice,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (60 * 65536));

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&this->MyDevice, 20000);
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&this->MyDevice,VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
        update_status(Status);

    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&this->MyDevice, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
        update_status(Status);

    }


   // GPIO.Init(this->statusLEDPin, Out_PP, SP_2MHz);
}

void LaserSensor::setAddress(uint8_t address)
{

    VL53L0X_SetDeviceAddress(&(this->MyDevice), (address * 2));

    this->MyDevice.I2cDevAddr = address;

}

int16_t LaserSensor::startRanging()
{


    VL53L0X_Error Status = Global_Status;
    bool startNow = true;



    if (Global_Status == VL53L0X_ERROR_NONE && startNow) {
        Status = VL53L0X_StartMeasurement(&this->MyDevice);
        update_status(Status);
        startNow = false;

    }



    if (Global_Status == VL53L0X_ERROR_NONE) {


        Status = VL53L0X_GetRangingMeasurementData(&this->MyDevice, &RangingMeasurementData);
        update_status(Status);

        isTofDataNewflag = true;
        Range_Status = RangingMeasurementData.RangeStatus;
        if (RangingMeasurementData.RangeStatus == 0) {
            this->range = (int16_t)RangingMeasurementData.RangeMilliMeter;


            out_of_range = false;
         //   GPIO.setLow(this->statusLEDPin);



        } else{

            this->range = -100;

            if(RangingMeasurementData.RangeStatus == 2)
            out_of_range = true;

           // GPIO.setLow(this->statusLEDPin);


        }

    } else{

        this->range = -100;
       // GPIO.setLow(this->statusLEDPin);
    }


    return this->range;

}

int16_t LaserSensor::getLaserRange()
{

    return this->range;
}

