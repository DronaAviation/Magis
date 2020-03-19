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



#include "API-Utils.h"
#include "Peripheral.h"
#include "XRanging.h"

LaserSensor laserLEFT;
LaserSensor laserRIGHT;
LaserSensor laserFRONT;
LaserSensor laserBACK;
LaserSensor laserEXTERNAL;

void XRanging_P::init(void)
{

    isXLaserInit[LEFT] = true;
    isXLaserInit[RIGHT] = true;
    isXLaserInit[FRONT] = true;
    isXLaserInit[BACK] = true;

}

void XRanging_P::init(laser_e laser)
{

    isXLaserInit[laser] = true;

}


int16_t XRanging_P::getRange(laser_e laser)
{

    switch (laser) {

    case LEFT:
        if (isXLaserInit[laser])

            return laserLEFT.startRanging();

        else
            return -1;

        break;

    case RIGHT:
        if (isXLaserInit[laser])

            return laserRIGHT.startRanging();

        else
            return -1;

        break;

    case FRONT:
        if (isXLaserInit[laser])

            return laserFRONT.startRanging();

        else
            return -1;

        break;

    case BACK:
        if (isXLaserInit[laser])

            return laserBACK.startRanging();

        else
            return -1;

        break;


    case EXTERNAL:
        if (isXLaserInit[laser])

            return laserEXTERNAL.startRanging();
//            return NewSensorRange;

        else
            return -1;

        break;

    }

}

void xRangingInit(void)
{

    uint8_t address = 42;

    if (isXLaserInit[LEFT]) {
        delay(10);

        GPIO.init(Pin15, OUTPUT); //LEFT
        GPIO.write(Pin15, STATE_LOW);

    }

    if (isXLaserInit[RIGHT]) {
        delay(10);

        GPIO.init(Pin13, OUTPUT); //RIGHT
        GPIO.write(Pin13, STATE_LOW);

    }

    if (isXLaserInit[FRONT]) {
        delay(10);
        GPIO.init(Pin10, OUTPUT);  // FRONT
        GPIO.write(Pin10, STATE_LOW);

    }

    if (isXLaserInit[BACK]) {
        delay(10);
        GPIO.init(Pin9, OUTPUT);  //BACK
        GPIO.write(Pin9, STATE_LOW);

    }


    if (isXLaserInit[EXTERNAL]){
//        delay(10);
//        GPIO.init(Pin8, OUTPUT);  //BACK
//        GPIO.write(Pin8, STATE_LOW);

    }


    if (isXLaserInit[LEFT]) {

        delay(30);

        GPIO.write(Pin15, STATE_HIGH);
        delay(30);

        laserLEFT.init();
        delay(30);

        laserLEFT.setAddress(address);

        address++;

    }

    if (isXLaserInit[RIGHT]) {

        delay(30);

        GPIO.write(Pin13, STATE_HIGH);
        delay(30);

        laserRIGHT.init();
        delay(30);
        laserRIGHT.setAddress(address);

        address++;
    }

    if (isXLaserInit[FRONT]) {

        delay(30);

        GPIO.write(Pin10, STATE_HIGH);
        delay(30);

        laserFRONT.init();
        delay(30);
        laserFRONT.setAddress(address);

        address++;

    }

    if (isXLaserInit[BACK]) {
        delay(30);

        GPIO.write(Pin9, STATE_HIGH);
        delay(30);

        laserBACK.init();
        delay(30);
        laserBACK.setAddress(address);

        address++;

    }


    if (isXLaserInit[EXTERNAL]) {

//        delay(30);
//
//        GPIO.write(Pin8, STATE_HIGH);
//        delay(30);

        laserEXTERNAL.init();
     //   useRangingSensor=true;
  //      delay(30);
//
//        laserEXTERNAL.setAddress(address);
//
//        address++;

    }



    delay(30);

}

XRanging_P XRanging;

