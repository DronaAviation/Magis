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

#include "vl53l1_platform.h"
//#include "vl53l0x_i2c_platform.h"

#include "vl53l1_api_core.h"
#include "vl53l1_api_strings.h"
#include "vl53l1_def.h"
#include "vl53l1_api.h"
#include "vl53l1_types.h"

#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/bus_i2c.h"
#include "drivers/system.h"
#include "ranging_vl53l1x.h"
#include "API/Peripheral.h"
#include "API/Utils.h"



#define LASER_LPS 0.1
#define RANGE_POLL 100
#define LASER_UPDATE_FREQUENCY 100000	//33ms

VL53L1_Dev_t MyDevice_L1;
//VL53L0X_Dev_t *pMyDevice = &MyDevice;

VL53L1_Error Global_Status_L1 = 0;
VL53L1_RangingMeasurementData_t RangingMeasurementData_L1;


uint8_t Range_Status_L1 = 0;
uint16_t NewSensorRange_L1 = 0;
uint16_t debug_range_L1 = 0;
bool isTofDataNewflag_L1 = false;
bool out_of_range_L1 = false;
bool startRanging_L1 = false;			//Cleanup later
bool useRangingSensor_L1 = false;		//Cleanup later

Interval rangePoll_L1;




void update_status_L1(VL53L1_Error Status)
{
    Global_Status_L1 = Status;
}

#ifdef LASER_TOF_L1x

void ranging_init_L1(void)
{
    VL53L1_Error Status = Global_Status_L1;
    VL53L1_DeviceInfo_t DeviceInfo;

    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    MyDevice_L1.I2cDevAddr = 0x29;
    MyDevice_L1.comms_type = 1;
    MyDevice_L1.comms_speed_khz = 400;

    Status = VL53L1_WaitDeviceBooted(&MyDevice_L1);	//Wait till the device boots. Blocking.

    if(Status == VL53L1_ERROR_NONE)
    	Status = VL53L1_DataInit(&MyDevice_L1); // Data initialization

    update_status_L1(Status);

    if(Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L1_StaticInit(&MyDevice_L1); // Device Initialization
        update_status_L1(Status);

    }

	/*if(Global_Status_L1 == VL53L1_ERROR_NONE) {
            Status = VL53L1_GetDeviceInfo(&MyDevice_L1, &DeviceInfo); // Device Initialization
            update_status_L1(Status);

    }
	*/

     /**/
    //Calibration is not necessary. Will look into this later. RefSpad, crosstalk and offset calibrations are available.

/*
    if( Global_Status_L1 == VL53L1_ERROR_NONE ) {
        Status = VL53L1_PerformRefSpadManagement( &MyDevice_L1); // Device Initialization
        update_status_L1(Status);

    }
/*
    if( Global_Status_L1 == VL53L1_ERROR_NONE ){
        Status = VL53L1_PerformOffsetCalibration( &MyDevice_L1, &VhvSettings, &PhaseCal ); // Device Initialization
        update_status_L1(Status);

    }

    if(Global_Status_L1 == VL53L1_ERROR_NONE){
        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        Status = VL53L0X_SetDeviceMode(&MyDevice_L1, VL53L0X_DEVICEMODE_SINGLE_RANGING);// Setup in single ranging mode
        update_status_L1(Status);

    }

    // Enable/Disable Sigma and Signal check
    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice_L1, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice_L1, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice_L1, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.1*65536));
        update_status_L1(Status);


    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice_L1, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(60*65536));

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&MyDevice_L1,33000);
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice_L1, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice_L1, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
        update_status_L1(Status);
    
    }
*/

}

void getRange_L1(uint32_t currentTime)
{
    VL53L1_Error Status = Global_Status_L1;
    static uint8_t dataFlag = 0, SysRangeStatus = 0;
    static bool startNow = true;
    static uint8_t print_counter;
    static uint32_t nextUpdateAt;
    VL53L1_State CurrPalState;

    if ((int32_t) (currentTime - nextUpdateAt) < 0)
           return;
       nextUpdateAt = currentTime + LASER_UPDATE_FREQUENCY;


    if(Global_Status_L1 == VL53L1_ERROR_NONE) {
        if(startNow) {
        	//Monitor.println("Hi");
            Status = VL53L1_StartMeasurement(&MyDevice_L1);
            update_status_L1(Status);
            /*if(Status)
            	Monitor.println(", Error = ", Global_Status_L1);
            	*/
            startNow = false;
        }
    }

    if(Global_Status_L1 == VL53L1_ERROR_NONE) {		//Check if data is ready
        if(!startNow) {

            Status = VL53L1_GetMeasurementDataReady(&MyDevice_L1,&dataFlag);
            update_status_L1(Status);
            //Monitor.println("Data flag ", dataFlag);
            //CurrPalState = VL53L1DevDataGet(MyDevice_L1, PalState);
            //Monitor.println("PAL ", MyDevice_L1.Data.LLData.measurement_mode);

        }
    }


    /**/
    if(Global_Status_L1 == VL53L1_ERROR_NONE) {
        if(dataFlag) {
        	Status = VL53L1_GetRangingMeasurementData(&MyDevice_L1, &RangingMeasurementData_L1);		//This is taking too much time
            update_status_L1(Status);
            if(RANGE_POLL > 90)
            	{
            		//Monitor.print("Timestamp ", RangingMeasurementData_L1.TimeStamp);
            		//Monitor.println("StreamCount : ", RangingMeasurementData_L1.StreamCount);
            		//Monitor.println("Range status : ", RangingMeasurementData_L1.RangeStatus);
            	}
            //Monitor.println(",   return status#4: ",Global_Status_L1);



            //startNow = true;
            isTofDataNewflag_L1 = true;
            Range_Status_L1 = RangingMeasurementData_L1.RangeStatus;
            if(RangingMeasurementData_L1.RangeStatus == 0) {

            	if(RangingMeasurementData_L1.RangeMilliMeter<4500){
              //  NewSensorRange_L1 = RangingMeasurementData.RangeMilliMeter;
            	if(RANGE_POLL > 90){
            		//Monitor.print("Ambient light ", RangingMeasurementData_L1.AmbientRateRtnMegaCps, 2);		//Ambient light
            		//Monitor.println(", Range is : ", RangingMeasurementData_L1.RangeMilliMeter);
            	}


                //NewSensorRange_L1 = NewSensorRange_L1*(1-LASER_LPS)+RangingMeasurementData_L1.RangeMilliMeter*LASER_LPS;	Why is this required
            	NewSensorRange_L1 = RangingMeasurementData_L1.RangeMilliMeter;
                out_of_range_L1 = false;


            	} else {
                    out_of_range_L1 = true;
				}

            } else
            out_of_range_L1 = true;
            dataFlag = 0;		//Reset the data flag
        }
    }
    /**/




}

bool isTofDataNew_L1(void)
{
    return isTofDataNewflag_L1;
}

bool isOutofRange_L1(void)
{
    return out_of_range_L1;
}

#endif

/*
void LaserSensor::init()
{



    VL53L1_Error Status = Global_Status_L1;

    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    this->MyDevice_L1.I2cDevAddr = 0x29;
    this->MyDevice_L1.comms_type = 1;
    this->MyDevice_L1.comms_speed_khz = 400;

   // this->statusLEDPin=pin;

    Status = VL53L0X_DataInit(&this->MyDevice_L1); // Data initialization

    update_status_L1(Status);

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_StaticInit(&this->MyDevice_L1); // Device Initialization
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_PerformRefSpadManagement(&this->MyDevice_L1, &refSpadCount, &isApertureSpads); // Device Initialization
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_PerformRefCalibration(&this->MyDevice_L1, &VhvSettings,&PhaseCal);           // Device Initialization
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        Status = VL53L0X_SetDeviceMode(&this->MyDevice_L1, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
        update_status_L1(Status);

    }

    // Enable/Disable Sigma and Signal check
    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&this->MyDevice_L1, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&this->MyDevice_L1, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&this->MyDevice_L1, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t) (0.1 * 65536));
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&this->MyDevice_L1,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (60 * 65536));

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&this->MyDevice_L1, 20000);
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&this->MyDevice_L1,VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
        update_status_L1(Status);

    }

    if (Global_Status_L1 == VL53L1_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&this->MyDevice_L1, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
        update_status_L1(Status);

    }


   // GPIO.Init(this->statusLEDPin, Out_PP, SP_2MHz);
}
/*
void LaserSensor::setAddress(uint8_t address)
{

    VL53L0X_SetDeviceAddress(&(this->MyDevice_L1), (address * 2));

    this->MyDevice_L1.I2cDevAddr = address;

}

int16_t LaserSensor::startRanging()
{


    VL53L1_Error Status = Global_Status_L1;
    bool startNow = true;



    if (Global_Status_L1 == VL53L1_ERROR_NONE && startNow) {
        Status = VL53L0X_StartMeasurement(&this->MyDevice_L1);
        update_status_L1(Status);
        startNow = false;

    }



    if (Global_Status_L1 == VL53L1_ERROR_NONE) {


        Status = VL53L0X_GetRangingMeasurementData(&this->MyDevice_L1, &RangingMeasurementData_L1);
        update_status_L1(Status);

        isTofDataNewflag_L1 = true;
        Range_Status_L1 = RangingMeasurementData_L1.RangeStatus;
        if (RangingMeasurementData_L1.RangeStatus == 0) {
            this->range = (int16_t)RangingMeasurementData_L1.RangeMilliMeter;


            out_of_range_L1 = false;
         //   GPIO.setLow(this->statusLEDPin);



        } else{

            this->range = -100;

            if(RangingMeasurementData_L1.RangeStatus == 2)
            out_of_range_L1 = true;

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
*/
