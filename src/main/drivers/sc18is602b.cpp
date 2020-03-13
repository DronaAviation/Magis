/*
 * This file is part of Magis.
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
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"


#include "API/Peripheral.h"
#include "API/Utils.h"


#include "sc18is602b.h"

void SC18IS602B::init(int slaveNum, bool a0, bool a1, bool a2) {
    //calculate the module's address here.
    //last 3 bit are the value of the address pin
    address = 0B0101000 | (a2 << 2) | (a1 << 1) | (a0);

    slaveNumber=slaveNum;
    slaveSelect = (1 << slaveNum);
}



void SC18IS602B::reset() {
    if(resetPin != -1) {
//        pinMode(resetPin, OUTPUT);
//        //RESET is low active, LOW
//        //Generate a high-to-low-to-high transition
//        //must be at least 50ns long (t_sa). 1ms is enough.
//        digitalWrite(resetPin, HIGH);
//        delay(1);
//        digitalWrite(resetPin, LOW);
//        delay(1);
//        digitalWrite(resetPin, HIGH);
    }
}

bool SC18IS602B::enableGPIO(int num, bool enable) {
    //sanity check
    if(num < 0 || num > 3)
        return false;
    //enable this GPIO while leaving the others untouched.
    bitWrite(gpioEnable, num, enable);
    //Send the new enable configuration
    return this->i2c_write(SC18IS601B_GPIO_ENABLE_CMD, &gpioEnable, sizeof(gpioEnable));
}

bool SC18IS602B::setupGPIO(int num, SC18IS601B_GPIOPinMode mode) {
    //sanity check
    if(num < 0 || num > 3)
        return false;

    //Cast the enum back to the bits
    //mode is a 2-bit wide bitfield
    uint8_t modeAsBitfield = (uint8_t) mode;

    //write 2 the bits into our last config value
    //refer to table 10 in the datasheet
    bitWrite(gpioConfig, 2*num, modeAsBitfield & 1);
    bitWrite(gpioConfig, 2*num + 1, modeAsBitfield >> 1);

    return this->i2c_write(SC18IS601B_GPIO_CONFIGURATION_CMD, &gpioConfig, sizeof(gpioConfig));
}

bool SC18IS602B::writeGPIO(int num, bool val) {
    if(num < 0 || num > 3)
        return false;
    //Re-write old value
    bitWrite(gpioWrite, num, val);
    return this->i2c_write(SC18IS601B_GPIO_WRITE_CMD, &gpioWrite, sizeof(gpioWrite));
}

bool SC18IS602B::writeGPIOBank(uint8_t value) {
    //remember new value
    gpioWrite = value;
    return this->i2c_write(SC18IS601B_GPIO_WRITE_CMD, &gpioWrite, sizeof(gpioWrite));
}


bool SC18IS602B::writeGPIOBank(bool gpio0, bool gpio1, bool gpio2, bool gpio3) {
    //Writes all gpio values to the pin
    uint8_t gpioVal = (gpio3 << 3) | (gpio2 << 2) | (gpio1 << 1) | gpio0;
    return writeGPIOBank(gpioVal);
}

bool SC18IS602B::readGPIO(int num) {
    if(num < 0 || num > 3)
        return false;

    //refer chapter 7.1.9
    //issue a read command.
    //this will cause the storage of 1 byte in the data buffer
   // if(!this->i2c_write(SC18IS601B_GPIO_READ_CMD, NUlLL, 0))
        return false;

    //Now try to read the buffer
    uint8_t gpioReadBuf = 0;
    size_t readBytes = this->i2c_read(&gpioReadBuf, sizeof(gpioReadBuf));

    if(readBytes == 0) {
        return false;
    }

    //return the bit at the needed position
    return bitRead(gpioReadBuf, num);
}

bool SC18IS602B::setLowPowerMode() {
  //  return this->i2c_write(SC18IS601B_IDLE_CMD, null, 0);
}

bool SC18IS602B::clearInterrupt() {
 //   return this->i2c_write(SC18IS601B_CLEAR_INTERRUPT_CMD, null, 0);
}

bool SC18IS602B::i2c_write(uint8_t cmdByte, uint8_t* data, uint8_t len) {
//    Wire.beginTransmission(address);
//    Wire.write(cmdByte);
//    Wire.write(data, len);
//    return Wire.endTransmission() == 0;


  //  return I2C.write(address, cmdByte, len, data);

}

bool SC18IS602B::configureSPI(bool lsbFirst, SC18IS601B_SPI_Mode spiMode,
        SC18IS601B_SPI_Speed clockSpeed) {
    //sanity check on parameters
    if(spiMode > SC18IS601B_SPIMODE_3)
        return false;
    uint8_t clk = (uint8_t)((uint8_t)(clockSpeed) & 0B11);

    //see chapter 7.1.5
    uint8_t configByte = (lsbFirst << 5) | (spiMode << 2) | clk;
    return this->i2c_write(SC18IS601B_CONFIG_SPI_CMD, &configByte, sizeof(configByte));
}


uint8_t SC18IS602B::spiTransfer(int slaveNum, uint8_t txByte) {
    uint8_t readBuf = 0;
    this->spiTransfer(slaveNum, &txByte, 1, &readBuf);
    return readBuf;
}

uint8_t SC18IS602B::i2c_read(uint8_t* readBuf, uint8_t len) {
//    while (Wire.requestFrom(address,len) == 0);
//    return Wire.readBytes(readBuf, len);

  //  return I2C.read(address,0xFF,len,readBuf);
}

bool SC18IS602B::spiTransfer(int slaveNum, uint8_t* txData, uint8_t txLen,
        uint8_t* readBuf) {
    //sanity check
    if(slaveNum < 0 || slaveNum > 3)
        return false;

    //Overly long data?
    if(txLen > SC18IS601B_DATABUFFER_DEPTH)
        return false;

    //the function ID will have the lower 4 bits set to the
    //activated slave selects. We use only 1 at a time here.
    uint8_t functionID = (1 << slaveNum);
    //transmit our TX buffer
    if(!this->i2c_write(functionID, txData, txLen))
        return false;


    delayMicroseconds(1000);

    //return  I2C.write(0x2F, functionID, txLen, txData);

    //read in the data that came from MISO
    return this->i2c_read(readBuf, txLen);

   // return I2C.read(0x2F,0xFF,2,readBuf);
   // return true;
}


bool SC18IS602B::write(uint8_t register_address,uint8_t data){

    bool success=false;

    uint8_t dataBuffer[2];

    dataBuffer[0]=register_address | 0x80u;

//    dataBuffer[0]=register_address;
    dataBuffer[1]=data;


   // success=I2C.write(address, slaveSelect, 2, dataBuffer);

    delay(1);

    return success;

}


uint8_t SC18IS602B::read(uint8_t register_address){

    uint8_t dataBuffer[2];

    dataBuffer[0]=register_address;
    dataBuffer[1]=0xFF;


    uint8_t readBuf[2];

    this->spiTransfer(slaveNumber, dataBuffer, 2, readBuf);

    return readBuf[1];

}


//uint8_t SC18IS602B::spiRead(uint8_t register_address,uint8_t data){
//
//    uint8_t dataBuffer[2];
//
//    dataBuffer[0]=register_address;
//    dataBuffer[1]=data;
//
//
//    return I2C.write(address, slaveSelect, 2, dataBuffer);
//
//}
