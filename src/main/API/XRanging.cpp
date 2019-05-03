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




void XRanging_P::init()
{

//    if(!isXLaserInit[LEFT]&&!isXLaserInit[RIGHT]&&!isXLaserInit[FRONT]&&!isXLaserInit[BACK]){
//
//    GPIO.init(Pin15, OUTPUT); //LEFT
//    GPIO.write(Pin15,STATE_LOW);
//
//    delay(10);
//    GPIO.init(Pin13, OUTPUT); //RIGHT
//     GPIO.write(Pin13,STATE_LOW);
//
//    delay(10);
//    GPIO.init(Pin10, OUTPUT);  // FRONT
//    GPIO.write(Pin10,STATE_LOW);
//
//    delay(10);
//    GPIO.init(Pin9, OUTPUT);  //BACK
//    GPIO.write(Pin9,STATE_LOW);
//
//    delay(30);

//    laser_sensors[0] = new LaserSensor();
//    laser_sensors[1] = new LaserSensor();
//    laser_sensors[2] = new LaserSensor();
//    laser_sensors[3] = new LaserSensor();


//    GPIO.write(Pin15,STATE_HIGH);
//    delay(30);
//
//    laserLEFT.init();
//    delay(30);
//
//    laserLEFT.setAddress(42);
//
//    delay(30);
//
//    GPIO.write(Pin13,STATE_HIGH);
//    delay(30);
//
//    laserRIGHT.init();
//    delay(30);
//    laserRIGHT.setAddress(43);
//
//    delay(30);
//
//
//    GPIO.write(Pin10,STATE_HIGH);
//    delay(30);
//
//    laserFRONT.init();
//    delay(30);
//    laserFRONT.setAddress(44);
//
//    delay(30);
//
//
//    GPIO.write(Pin9,STATE_HIGH);
//    delay(30);
//
//    laserBACK.init();
//    delay(30);
//    laserBACK.setAddress(45);
//
//    delay(30);


    isXLaserInit[LEFT]=true;
    isXLaserInit[RIGHT]=true;
    isXLaserInit[FRONT]=true;
    isXLaserInit[BACK]=true;

//    }
}


void XRanging_P::init(laser_e laser)
{

    isXLaserInit[laser]=true;

//
//    switch(laser){
//
//
//    case LEFT:
//
//        if(!isXLaserInit[LEFT]){
//
//        delay(10);
//
//        GPIO.init(Pin15, OUTPUT);
//        GPIO.write(Pin15,STATE_LOW);
//
//        delay(30);
//
//
// //       laser_sensors[LEFT] = new LaserSensor();
//
//        GPIO.write(Pin15,STATE_HIGH);
//        delay(30);
//
//        laserLEFT.init();
//        delay(30);
//        laserLEFT.setAddress(42);
//
//        isXLaserInit[LEFT]=true;
//
//        }
//
//        break;
//
//
//
//    case RIGHT:
//
//        if(!isXLaserInit[RIGHT]){
//
//        delay(10);
//
//        GPIO.init(Pin13, OUTPUT);
//        GPIO.write(Pin13,STATE_LOW);
//
//        delay(30);
//
//  //      laser_sensors[RIGHT] = new LaserSensor();
//
//        GPIO.write(Pin13,STATE_HIGH);
//        delay(30);
//
//        laserRIGHT.init();
//        delay(30);
//        laserRIGHT.setAddress(43);
//
//        isXLaserInit[RIGHT]=true;
//
//
//        }
//
//
//        break;
//
//
//
//    case FRONT:
//
//        if(!isXLaserInit[FRONT]){
//
//        delay(10);
//
//        GPIO.init(Pin10, OUTPUT);
//        GPIO.write(Pin10,STATE_LOW);
//
//        delay(30);
//
//
// //       laser_sensors[FRONT] = new LaserSensor();
//
//        GPIO.write(Pin10,STATE_HIGH);
//        delay(30);
//
//        laserFRONT.init();
//        delay(30);
//        laserFRONT.setAddress(44);
//
//        isXLaserInit[FRONT]=true;
//
//        }
//
//
//        break;
//
//
//
//
//    case BACK:
//
//        if(!isXLaserInit[LEFT]){
//
//        delay(10);
//
//        GPIO.init(Pin9, OUTPUT);
//        GPIO.write(Pin9,STATE_LOW);
//
//        delay(30);
//
//
// //       laser_sensors[BACK] = new LaserSensor();
//
//        GPIO.write(Pin9,STATE_HIGH);
//        delay(30);
//
//        laserBACK.init();
//        delay(30);
//        laserBACK.setAddress(45);
//
//        isXLaserInit[BACK]=true;
//
//        }
//
//        break;
//
//
//
//
//
//
//
//
//
//
//
//    }
//

}


void XRanging_P::start()
{

    startShieldRanging = true;

}

void XRanging_P::stop()
{

    startShieldRanging = false;

}

int16_t XRanging_P::getRange(laser_e laser)
{

    switch(laser){

    case LEFT:
            if(isXLaserInit[laser])

                return laserLEFT.startRanging();

            else
                return -1;

            break;

    case RIGHT:
            if(isXLaserInit[laser])

                return laserRIGHT.startRanging();

            else
                return -1;

            break;


    case FRONT:
            if(isXLaserInit[laser])

                return laserFRONT.startRanging();

            else
                return -1;

            break;



    case BACK:
            if(isXLaserInit[laser])

                return laserBACK.startRanging();

            else
                return -1;

            break;

    }

}



void xRangingInit(){


    uint8_t address=42;


    if(isXLaserInit[LEFT]){
        delay(10);

    GPIO.init(Pin15, OUTPUT); //LEFT
    GPIO.write(Pin15,STATE_LOW);


    }


    if(isXLaserInit[RIGHT]){
        delay(10);

    GPIO.init(Pin13, OUTPUT); //RIGHT
     GPIO.write(Pin13,STATE_LOW);



    }


    if(isXLaserInit[FRONT]){
        delay(10);
    GPIO.init(Pin10, OUTPUT);  // FRONT
    GPIO.write(Pin10,STATE_LOW);



    }


    if(isXLaserInit[BACK]){
    delay(10);
    GPIO.init(Pin9, OUTPUT);  //BACK
    GPIO.write(Pin9,STATE_LOW);

    }



    if(isXLaserInit[LEFT]){

    delay(30);

    GPIO.write(Pin15,STATE_HIGH);
    delay(30);

    laserLEFT.init();
    delay(30);

    laserLEFT.setAddress(address);

    address++;

    }


    if(isXLaserInit[RIGHT]){

    delay(30);

    GPIO.write(Pin13,STATE_HIGH);
    delay(30);

    laserRIGHT.init();
    delay(30);
    laserRIGHT.setAddress(address);

    address++;
    }


    if(isXLaserInit[FRONT]){

    delay(30);


    GPIO.write(Pin10,STATE_HIGH);
    delay(30);

    laserFRONT.init();
    delay(30);
    laserFRONT.setAddress(address);

    address++;

    }


    if(isXLaserInit[BACK]){
    delay(30);


    GPIO.write(Pin9,STATE_HIGH);
    delay(30);

    laserBACK.init();
    delay(30);
    laserBACK.setAddress(address);

    address++;

    }



    delay(30);



}



XRanging_P XRanging;

