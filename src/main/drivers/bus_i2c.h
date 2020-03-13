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

typedef enum I2CDevice {
    I2CDEV_1,
    I2CDEV_2,
    I2CDEV_MAX = I2CDEV_2,
} I2CDevice;

void i2cInit(I2CDevice index);
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data);
uint8_t i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);

bool i2cWriteBufferwithoutregister(uint8_t addr_,uint8_t len_, uint8_t *data);
bool i2cWritewithoutregister(uint8_t addr_,  uint8_t data);
uint8_t i2cReadwithoutregister(uint8_t addr_, uint8_t len, uint8_t* buf);

uint8_t i2cReadDevice(uint8_t addr_, uint8_t len, uint8_t* buf);

uint16_t i2cGetErrorCounter(void);

#ifdef __cplusplus
}
#endif 
