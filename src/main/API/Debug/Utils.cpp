/*
 * utils.cpp
 *
 *  Created on: 05-Aug-2017
 *      Author: User
 */


#include <stdint.h>

#include "platform.h"

#include "common/maths.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "Utils.h"


uint32_t microsT(void)
{
return micros();

}


Timer::Timer()
{

time=0;
loopTime=0;

}



bool Timer::start(uint32_t time)
{




if(this->time==0)
{

	this->time=constrain(time, 1, 5000);
	this->loopTime=millis()+this->time;
}


if((int32_t)(millis()-this->loopTime)>=0)
{
	loopTime=millis()+this->time;
    return true;

}

return false;

}


void Timer::reset(void)
{


    this->time=0;
    this->loopTime=0;


}
