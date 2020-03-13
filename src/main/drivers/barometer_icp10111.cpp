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
#define MS5611_ADDR                 0x77

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

static void ms5611_reset(void);
static uint16_t ms5611_prom(int8_t coef_num);
STATIC_UNIT_TESTED int8_t ms5611_crc(uint16_t *prom);
static uint32_t ms5611_read_adc(void);
static void ms5611_start_ut(void);
static uint32_t ms5611_get_ut(void);
static void ms5611_start_up(void);
static uint32_t ms5611_get_up(void);
STATIC_UNIT_TESTED void ms5611_calculate(float *pressure, float *temperature, uint32_t P, uint32_t T);
static bool icp10111_read(uint32_t currentTime,float *pressure, float *temperature);
void icp10111_calculate(float *pressure, float *temperature,uint32_t raw_p,uint16_t raw_t);
static uint32_t icp10111_measureStart(mmode mode);

STATIC_UNIT_TESTED uint32_t ms5611_ut;  // static result of temperature measurement
STATIC_UNIT_TESTED uint32_t ms5611_up;  // static result of pressure measurement
STATIC_UNIT_TESTED uint16_t ms5611_c[PROM_NB];  // on-chip ROM
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


#define ICP_I2C_ID 0x63

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

    ack=i2cWriteBufferwithoutregister(0x63, 2, command );
//
//    i2cReadwithoutregister(0x63, 0x00, 2, buf);


   // ack=i2cWrite(0x63,   command[0],  command[1] );

//    while(1){

  //  ack=i2cWrite(0x63,   0xef,  0xc8 );

    delay(1);

i2cReadwithoutregister(0x63,  2, buf);

  //  i2cRead(0x63, 0xff, 2, buf);

//    delay(3);
//
//    }

 //  i2cRead(0x63, 0xff, 2, buf);

  //  i2cReadwithoutregister(0x63, 0x00, 2, buf);

    uint16_t id = (buf[0] << 8) | buf[1];


    if (ack) {


        if ((id & 0x03f) == 0x08) {

        //  LED_M_ON;

            //  while(1);
            //return true;

        } else {

//          LED_R_ON;
//
//          baroRead=id;
//          while(1);
//          return true;
        }


        //return true;
    } else {

//      LED_L_ON;
//      while(1);
//      return false;
    }

//    ack = i2cRead(0X63, CMD_PROM_RD, 1, &sig);
//    if (!ack) {
//
//        return false;
//
//    }

//    ms5611_reset();
//    // read all coefficients
//    for (i = 0; i < PROM_NB; i++)
//        ms5611_c[i] = ms5611_prom(i);
//    // check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
//    if (ms5611_crc(ms5611_c) != 0) {
//
//        return false;
//
//    }


    // read sensor calibration data
    uint8_t addr_otp_cmd[5] = {
                (ICP_CMD_SET_ADDR >> 8) & 0xff,
                ICP_CMD_SET_ADDR & 0xff,
                0x00, 0x66, 0x9c };
    uint8_t otp_buf[3];
    i2cWriteBufferwithoutregister(0x63,5,addr_otp_cmd);

    for (int i=0; i<4; i++) {

        command[0]=(ICP_CMD_READ_OTP >> 8) & 0xff;
        command[1]=ICP_CMD_READ_OTP & 0xff;

        i2cWriteBufferwithoutregister(0x63, 2, command );

        //i2cWritewithoutregister(0x63,ICP_CMD_READ_OTP);

        //_readResponse(otp_buf, 3);

        delay(1);

     i2cReadwithoutregister(0x63,  2, otp_buf);

        _scal[i] = (otp_buf[0] << 8) | otp_buf[1];
    }

    //baroRead=_scal[0];


    // TODO prom + CRC
//    baro->ut_delay = 10000;
//    baro->up_delay = 10000;
//    baro->start_ut = ms5611_start_ut;
//    baro->get_ut = ms5611_get_ut;
//    baro->start_up = ms5611_start_up;
//    baro->get_up = ms5611_get_up;
//    baro->calculate = ms5611_calculate;
    baro->measurment_start=icp10111_measureStart;
    baro->read=icp10111_read;


    _pressure_Pa = 0.0;
    _temperature_C = 0.0;
    _meas_duration = 98;
    _meas_start = millis();
    _data_ready = false;

   // baro->measurment_start(NORMAL);
    return true;
}

static void ms5611_reset(void)
{
    i2cWrite(MS5611_ADDR, CMD_RESET, 1);
    delayMicroseconds(2800);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
    uint8_t rxbuf[2] = { 0, 0 };
    i2cRead(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf); // send PROM READ command
    return rxbuf[0] << 8 | rxbuf[1];
}

STATIC_UNIT_TESTED int8_t ms5611_crc(uint16_t *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    bool blankEeprom = true;

    for (i = 0; i < 16; i++) {
        if (prom[i >> 1]) {
            blankEeprom = false;
        }
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (!blankEeprom && crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}

static uint32_t ms5611_read_adc(void)
{
    uint8_t rxbuf[3];
    i2cRead(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf); // read ADC
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static void ms5611_start_ut(void)
{
    i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

static uint32_t ms5611_get_ut(void)
{
    ms5611_ut = ms5611_read_adc();

    return ms5611_ut;
}

static void ms5611_start_up(void)
{
    i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

static uint32_t ms5611_get_up(void)
{
    ms5611_up = ms5611_read_adc();

    return ms5611_up;

}

STATIC_UNIT_TESTED void ms5611_calculate(float *pressure, float *temperature, uint32_t P, uint32_t T)
{

    uint32_t press;
    int64_t temp;
    int64_t delt;

    int64_t dT = (int64_t) T - ((uint64_t) ms5611_c[5] * 256);
    int64_t off = ((int64_t) ms5611_c[2] << 16) + (((int64_t) ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t) ms5611_c[1] << 15) + (((int64_t) ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t) ms5611_c[6]) >> 23);

    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
        temp -= ((dT * dT) >> 31);
    }

    press = ((((int64_t) P * sens) >> 21) - off) >> 15;

    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;

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

   i2cWriteBufferwithoutregister(0x63, 2, command );
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
