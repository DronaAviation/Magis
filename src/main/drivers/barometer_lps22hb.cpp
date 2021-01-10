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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "barometer.h"

#include "gpio.h"
#include "system.h"
#include "bus_i2c.h"

#include "build_config.h"
#include "config/runtime_config.h"
#include "drivers/light_led.h"
#include "drivers/system.h"

#include "barometer_lps22hb.h"


// MS5611, Standard address 0x77
#define LPS22HB_ADDR             0x5C

#define WHO_AM_I			0x0F // Who am I?
#define CTRL_REG1				0x10 //
#define CTRL_REG2				0x11 //
#define CTRL_REG3				0x12 //
#define RESPONSE_WHO_AM_I		0xB1 //Response of who am I
#define PRESS_OUT_XL			0x28 //Output pressure Lowest byte
#define TEMP_OUT_L				0x2B //Output temperature Lowest byte
#define STATUS					0x27 //Status of readings

#define PRESS_SENSITIVITY		40.96 //Scale factor for pressure
#define TEMP_SENSITIVITY		100 //Scale factor for temperature

static uint32_t lps22hb_get_ut(void);
static uint32_t lps22hb_get_up(void);
bool lps22hb_press_ready(void);
bool lps22hb_temp_ready(void);
static bool lps22hb_calculate(uint32_t _currentTime, float *_baroPressure, float *_baroTemperature);


bool lps22hbDetect(baro_t *baro)
{
    bool ack = false;
    uint8_t sig;
    uint8_t temp_reg;
    int i;

    delay(10); // No idea how long the chip takes to power-up, but let's make it 10ms

    ack = i2cRead(LPS22HB_ADDR, WHO_AM_I, 1, &sig);
    if (ack && (sig == RESPONSE_WHO_AM_I)) {
    	//Configuration of baro

    	delay(1);
    	temp_reg = 0;
    	temp_reg = 3 << 4 | 1 << 3 | 1 << 1; 		// ODR 25Hz, Enable LPF, BDU = 1,
    	//temp_reg = 2 << 4 | 1 << 3 | 1 << 2| 1 << 3| 1 << 1; 		// ODR 25Hz, Enable LPF, Bandwidth, BDU = 1,
    	ack = i2cWrite(LPS22HB_ADDR, CTRL_REG1, temp_reg);

    	delay(1);
    	if(ack){
    		temp_reg = 0;
    	    temp_reg = 1 << 4; 							// Auto increment enable
    	    ack = i2cWrite(LPS22HB_ADDR, CTRL_REG2, temp_reg);


    	}else{
    	    return false;
    	}
    }
    else{
    	return false;
    }

    //Pressure and temperature read functions
    baro->up_delay = 40000;			//40ms
    baro->get_ut = lps22hb_get_ut;
    baro->get_up = lps22hb_get_up;
    baro->pressure_rdy = lps22hb_press_ready;
    baro->temperature_rdy = lps22hb_temp_ready;
    baro->read = lps22hb_calculate;

    return true;
}
/*bool lps22hbInit(void){


}*/


static uint32_t lps22hb_get_ut(void)
{	//Get Temperature
	uint32_t baro_temp;
	uint8_t buffer[2];

	i2cRead(LPS22HB_ADDR, TEMP_OUT_L, 2, buffer);

    baro_temp = buffer[1] << 8 | buffer[0];

    return baro_temp;
}

static uint32_t lps22hb_get_up(void)
{	//Get Pressure
	uint32_t baro_pressure;
	uint8_t buffer[3];

	i2cRead(LPS22HB_ADDR, PRESS_OUT_XL, 3, buffer);

    baro_pressure = buffer[2] << 16 | buffer[1] << 8 | buffer[0];

    return baro_pressure;
}

bool lps22hb_press_ready(void){
	uint8_t status, result;

	i2cRead(LPS22HB_ADDR, STATUS, 1, &status);
	result =  status & 0x01;
	return result;
}

bool lps22hb_temp_ready(void){

	uint8_t status, result;

	i2cRead(LPS22HB_ADDR, STATUS, 1, &status);
	result =  status & 0x02;
	return result;

}

static bool lps22hb_calculate(uint32_t _currentTime, float *_baroPressure, float *_baroTemperature){
	float raw_baro_pressure, raw_baro_temperature;
	if(lps22hb_press_ready()){
	    	raw_baro_pressure = (float)lps22hb_get_up();
	    	*_baroPressure = raw_baro_pressure / ((float)PRESS_SENSITIVITY);		//Pressure in Pa
	    }

	    if(lps22hb_temp_ready()){
	    	raw_baro_temperature = (float)lps22hb_get_ut();
	    	*_baroTemperature = raw_baro_temperature / TEMP_SENSITIVITY;
	    }

	    return true;
}
