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

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "build_config.h"

#include "platform.h"

#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"

//#include "config/parameter_group.h"

#include "system.h"
#include "gpio.h"
#include "light_led.h"
#include "exti.h"
#include "bus_i2c.h"
#include "bus_spi.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#include "sensor.h"
#include "compass.h"

#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"
#include "accgyro_spi_mpu6500.h"
#include "compass_ak09916.h"

// This sensor is available in MPU-9250.
#ifndef AK8963_I2C_INSTANCE
#define AK8963_I2C_INSTANCE I2C_DEVICE
#endif

// AK8963, mag sensor address
#define AK8963_MAG_I2C_ADDRESS          0x0C
#define AK8963_Device_ID                0x48

// Registers
#define AK8963_MAG_REG_WHO_AM_I         0x00
#define AK8963_MAG_REG_INFO             0x01
#define AK8963_MAG_REG_STATUS1          0x02
#define AK8963_MAG_REG_HXL              0x03
#define AK8963_MAG_REG_HXH              0x04
#define AK8963_MAG_REG_HYL              0x05
#define AK8963_MAG_REG_HYH              0x06
#define AK8963_MAG_REG_HZL              0x07
#define AK8963_MAG_REG_HZH              0x08
#define AK8963_MAG_REG_STATUS2          0x09
#define AK8963_MAG_REG_CNTL             0x0a
#define AK8963_MAG_REG_ASCT             0x0c // self test
#define AK8963_MAG_REG_ASAX             0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAY             0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_MAG_REG_ASAZ             0x12 // Fuse ROM z-axis sensitivity adjustment value

#define READ_FLAG                       0x80

#define STATUS1_DATA_READY              0x01
#define STATUS1_DATA_OVERRUN            0x02

#define STATUS2_DATA_ERROR              0x02
#define STATUS2_MAG_SENSOR_OVERFLOW     0x03

#define CNTL_MODE_POWER_DOWN            0x00
#define CNTL_MODE_ONCE                  0x01
#define CNTL_MODE_CONT1                 0x02
#define CNTL_MODE_CONT2                 0x06
#define CNTL_MODE_SELF_TEST             0x08
#define CNTL_MODE_FUSE_ROM              0x0F

static float magGain[3] = { 1.0f, 1.0f, 1.0f };


bool ak09916SensorRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cRead(addr_, reg_, len, buf);
}

bool ak09916SensorWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWrite(addr_, reg_, data);
}



bool ak09916SensorMagRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{

    ak09916SensorWrite(0x68, 0x7F, 0x30);

    delay(1);

   ak09916SensorWrite(0x68, 0x03 ,addr_|0x80);

    delay(1);

    ak09916SensorWrite(0x68, 0x04 ,reg_);


    delay(1);

   // ak8963SensorWrite(0x68,0x06 ,0xff);
    ak09916SensorWrite(0x68,0x05 ,(0x80|len));

    delay(1);

    ak09916SensorWrite(0x68,0x7F, 0x00);
    delay(1);

    bool ack = ak09916SensorRead(0x68, 0x3B, len, buf);

    return ack;
}

bool ak09916SensorMagWrite(uint8_t reg_, uint8_t data)
{

    ak09916SensorWrite(0x68,0x7F, 0x30);

    delay(1);
    ak09916SensorWrite(0x68,0x03 ,0x0C);//mode: write

    delay(1);
    ak09916SensorWrite(0x68,0x04 ,reg_);//set reg addr

    delay(1);
    bool ack=ak09916SensorWrite(0x68,0x06 ,data);//send value

    //delay(1);


    return ack;
}




bool ak09916Detect(mag_t *mag)
{
    bool ack = false;
    uint8_t sig =0;


    // Configure AUX_I2C Magnetometer (onboard ICM-20948)
    ak09916SensorWrite(0x68,0x7F, 0x00); // Select user bank 0
    ak09916SensorWrite(0x68,0x0F, 0x32); // INT Pin / Bypass Enable Configuration

    ak09916SensorWrite(0x68,0x03, 0x00); // I2C_MST_EN - disable i2c master

    /*
    ak09916SensorWrite(0x68,0x7F, 0x30); // Select user bank 3
    ak09916SensorWrite(0x68,0x01, 0x4D); // I2C Master mode and Speed 400 kHz
    ak09916SensorWrite(0x68,0x02, 0x01); // I2C_SLV0 _DLY_ enable
    ak09916SensorWrite(0x68,0x05, 0x81); // enable IIC   and EXT_SENS_DATA==1 Byte
     */


    ack = ak09916SensorRead(0x0C, 0x01, 1, &sig);


    int16_t deviceId=sig;




    if ( deviceId == 0x09)// 0x48 / 01001000 / 'H'
   {
        mag->init = (sensorInitFuncPtr) ak09916Init;
        mag->read = (sensorReadFuncPtr)  ak09916Read;

        return true;
    }


    return false;
}

bool ak09916Init()
{
    bool ack;
    UNUSED(ack);

    uint8_t status;

    ak09916SensorWrite(0x0C, 0x32, 0x01); // Reset AK8963


    delay(1000);
    ak09916SensorWrite(0X0C,0x31, 0x01);

    delay(1);


    return true;
}

bool ak09916Read(int16_t *magData)
{
    bool ack = false;
    uint8_t buf[8];


    ack = ak09916SensorRead(AK8963_MAG_I2C_ADDRESS, 0x10, 1, buf);


    uint8_t status = buf[0];


    if (!ack || ((status  & 0x01) == 0x00)) {

        return false;
    }


    ack = ak09916SensorRead(AK8963_MAG_I2C_ADDRESS, 0x11, 7, buf);


    uint8_t status2 = buf[6];



    if (!ack) {

        return false;
    }

    magData[X] = (int16_t) (buf[1] << 8 | buf[0]) * magGain[X];
    magData[Y] = (int16_t) (buf[3] << 8 | buf[2]) * magGain[Y];
    magData[Z] = (int16_t) (buf[5] << 8 | buf[4]) * magGain[Z];



    return ak09916SensorWrite(AK8963_MAG_I2C_ADDRESS,0x31, 0x01); // start reading again

}
