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

#include "barometer_icp10111.h"

// MS5611, Standard address 0x77
#define ICP101xx_ADDR           0x77
/*
#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
*/

static bool icp10111_read(uint32_t currentTime,float *pressure, float *temperature);
void icp10111_calculate(float *pressure, float *temperature,uint32_t raw_p,uint16_t raw_t);
static uint32_t icp10111_measureStart(mmode mode);

static uint8_t ms5611_osr = CMD_ADC_4096;

uint32_t baroRead=9230;


float _scal[4];
uint16_t _raw_t;
uint32_t _raw_p;
float _temperature_C;
float _pressure_Pa;
uint32_t _meas_start;
uint32_t _meas_duration;
bool _data_ready;

#define ICP_CMD_READ_ID 0xefc8
#define ICP_CMD_SET_ADDR 0xc595
#define ICP_CMD_READ_OTP 0xc7f7
#define ICP_CMD_MEAS_LP 0x609c
#define ICP_CMD_MEAS_N 0x6825
#define ICP_CMD_MEAS_LN 0x70df
#define ICP_CMD_MEAS_ULN 0x7866

// constants for presure calculation
const float _pcal[3] = { 45000.0, 80000.0, 105000.0 };
const float _lut_lower = 3.5 * 0x100000;    // 1<<20
const float _lut_upper = 11.5 * 0x100000;   // 1<<20
const float _quadr_factor = 1 / 16777216.0;
const float _offst_factor = 2048.0;



void sendCommand(int16_t command){

    //i2

}

bool icp10111Detect(baro_t *baro)
{
    bool ack = false;
    uint8_t sig;
    int i;
    uint8_t command[2];

    uint8_t buf[3];

    delay(10); // No idea how long the chip takes to power-up, but let's make it 10ms


    command[0]=(0xefc8 >> 8) & 0xff;
    command[1]=0xefc8 & 0xff;

    ack=i2cWriteBufferwithoutregister(ICP101xx_ADDR, 2, command );

    delay(1);

    i2cReadwithoutregister(ICP101xx_ADDR,  2, buf);
    uint16_t id = (buf[0] << 8) | buf[1];

    if (ack) {

        if ((id & 0x03f) == 0x08) {
        	//Do nothing
        } else {
        	return false;
        }
    } else {
    	return false;
    }

    uint8_t addr_otp_cmd[5] = {
                (ICP_CMD_SET_ADDR >> 8) & 0xff,
                ICP_CMD_SET_ADDR & 0xff,
                0x00, 0x66, 0x9c };
    uint8_t otp_buf[3];
    i2cWriteBufferwithoutregister(ICP101xx_ADDR,5,addr_otp_cmd);

    for (int i=0; i<4; i++) {

        command[0]=(ICP_CMD_READ_OTP >> 8) & 0xff;
        command[1]=ICP_CMD_READ_OTP & 0xff;

        i2cWriteBufferwithoutregister(ICP101xx_ADDR, 2, command );

        delay(1);

        i2cReadwithoutregister(ICP101xx_ADDR,  2, otp_buf);

        _scal[i] = (otp_buf[0] << 8) | otp_buf[1];
    }

    baro->measurment_start=icp10111_measureStart;
    baro->read=icp10111_read;

    _pressure_Pa = 0.0;
    _temperature_C = 0.0;
    _meas_duration = 98;
    _meas_start = millis();
    _data_ready = false;


    return true;
}


static uint32_t icp10111_measureStart(mmode mode) {
    uint16_t cmd;
    uint8_t command[2];
    switch (mode) {
    case FAST:
        cmd = ICP_CMD_MEAS_LP;
        _meas_duration = 3;
        break;
    case ACCURATE:
        cmd = ICP_CMD_MEAS_LN;
        _meas_duration = 24;
        break;
    case VERY_ACCURATE:
        cmd = ICP_CMD_MEAS_ULN;
        _meas_duration = 98;
        break;
    case NORMAL:
    default:
        cmd = ICP_CMD_MEAS_N;
        _meas_duration = 7;
        break;
    }
//  _sendCommand(cmd);

    command[0]=(cmd >> 8) & 0xff;
    command[1]=cmd & 0xff;

   i2cWriteBufferwithoutregister(ICP101xx_ADDR, 2, command );
   // baroRead++;
    _data_ready = false;
    _meas_start = millis();
    return _meas_duration;
}

void icp10111_calculate(float *pressure, float *temperature,uint32_t raw_p,uint16_t raw_t){

    // calculate temperature
    _temperature_C = -45.f + 175.f / 65536.f * raw_t;

    // calculate pressure
    float t = (float)(raw_t - 32768);
    float s1 = _lut_lower + (float)(_scal[0] * t * t) * _quadr_factor;
    float s2 = _offst_factor * _scal[3] + (float)(_scal[1] * t * t) * _quadr_factor;
    float s3 = _lut_upper + (float)(_scal[2] * t * t) * _quadr_factor;
    float c = (s1 * s2 * (_pcal[0] - _pcal[1]) +
               s2 * s3 * (_pcal[1] - _pcal[2]) +
               s3 * s1 * (_pcal[2] - _pcal[0])) /
              (s3 * (_pcal[0] - _pcal[1]) +
               s1 * (_pcal[1] - _pcal[2]) +
               s2 * (_pcal[2] - _pcal[0]));
    float a = (_pcal[0] * s1 - _pcal[1] * s2 - (_pcal[1] - _pcal[0]) * c) / (s1 - s2);
    float b = (_pcal[0] - a) * (s1 + c);
    _pressure_Pa = a + b / (c + raw_p);

//  baroRead=_raw_t;
    if (pressure)
        *pressure = _pressure_Pa;
    if (temperature)
        *temperature = _temperature_C;

  //  baroRead=_pressure_Pa;
}


static bool icp10111_read(uint32_t currentTime,float *pressure, float *temperature){


//   baroRead=_meas_duration;

    if (_data_ready)
        return true;

     //=(currentTime/1000);



    if ((currentTime - _meas_start) < _meas_duration){
//      baroRead=0;
        return false;
    }

    //baroRead=currentTime - _meas_start;

    uint8_t res_buf[9];
    //_readResponse(res_buf, 9);


    i2cReadwithoutregister(0x63,  9, res_buf);
    _raw_t = (res_buf[0] << 8) | res_buf[1];
    uint32_t L_res_buf3 = res_buf[3];   // expand result bytes to 32bit to fix issues on 8-bit MCUs
    uint32_t L_res_buf4 = res_buf[4];
    uint32_t L_res_buf6 = res_buf[6];
    _raw_p = (L_res_buf3 << 16) | (L_res_buf4 << 8) | L_res_buf6;
//  _calculate();

    //baroRead=_raw_t;

//  baroRead++;
    icp10111_calculate(pressure,temperature,_raw_p, _raw_t);

    _data_ready = true;

    return true;



}
