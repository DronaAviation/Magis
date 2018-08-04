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

#include "Xshield.h"

#include "../API-Utils.h"
#include "../Hardware/Peripheral.h"

void Xshield_P::init()
{

    GPIO.Init(Pin15, Out_PP, SP_2MHz);
    GPIO.setLow(Pin15);

    delay(10);
    GPIO.Init(Pin13, Out_PP, SP_2MHz);
    GPIO.setLow(Pin13);

    delay(10);
    GPIO.Init(Pin14, Out_PP, SP_2MHz);
    GPIO.setLow(Pin14);

    delay(30);

    laser_sensors[0] = new LaserSensor();
    laser_sensors[1] = new LaserSensor();


    GPIO.setHigh(Pin13);
    delay(30);

    laser_sensors[0]->init();
    delay(30);
    laser_sensors[0]->setAddress(42);

    delay(30);

    GPIO.setHigh(Pin15);
    delay(30);

    laser_sensors[1]->init();
    delay(30);
    laser_sensors[1]->setAddress(43);

    delay(30);


}

void Xshield_P::startRanging()
{

    startShieldRanging = true;

}

void Xshield_P::stopRanging()
{

    startShieldRanging = false;

}

uint16_t Xshield_P::getRange(laser_e range)
{

    return laser_sensors[range]->getLaserRange();

}

Xshield_P Xshield;

