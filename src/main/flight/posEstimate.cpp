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
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "io/serial_msp.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "config/runtime_config.h"

#include "posEstimate.h"


#include "posControl.h"

#define corr_scale 512/100
#define corr_scale2 1096/100
#define MILLI_SEC 1000
#define SEC 1000000

static pidProfile_t *pidProfile; //PS2

int16_t print_posvariable1 = 0;
int16_t print_posvariable2 = 0;
int16_t print_posvariable3 = 0;
int16_t print_posvariable4 = 0;
int16_t print_posvariable5 = 0;
int16_t print_posvariable6 = 0;

int16_t posX = 0;
int16_t posY = 0;
int16_t posZ = 0;
int16_t deltaTime = 0;

int8_t Quality=-1;
bool new_position=false;
int8_t localisationType = -1;

float inputX = 0.0f;
float inputY = 0.0f;
float dTime = 0.0f;
float inputZ = 0.0f; //

float inputPreX = 0.0f;
float inputPreY = 0.0f;

static int16_t first_read = 0;
int16_t i;

float accel_EF[2];
float accel_EF_correction[2];
float est_velocity[2];
float accel_EF_prev[2];
float position_error[2];
float position_base[2];
float position_correction[2];
float est_position[2];
float velocity_increase[2];

uint8_t hist_xy_counter; // counter used to slow saving of position estimates for later comparison
float hist_position_baseX;
float hist_position_baseY;

float time_constant_xy = 1.5f;  // can be tuned
float k1;
float k2;
float k3;

float PositionX = 0;
float PositionY = 0;
float PrevPositionX = 0;
float PrevPositionY = 0;
int16_t VelocityX = 0;
int16_t VelocityY = 0;
int16_t inputVx = 0;
int16_t inputVy = 0;


#define BUFFER_XY_MAXIMUM 6 //can be tuned
#define INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS 10

static float bufferX[BUFFER_XY_MAXIMUM], bufferY[BUFFER_XY_MAXIMUM];
static int16_t head = 0;
static int16_t rear = -1;
static int16_t itemCount = 0;

void resetPosition(void)
{       //Reset Pos and velocity
    est_position[0] = 0;
    est_position[1] = 0;
    est_velocity[0] = 0;
    est_velocity[1] = 0;
    resetPosIntegral();
}


void addHistPositionBaseEstXY(float positionX, float positionY){
    // determine position of new item
    rear = head + itemCount;
    if( rear >= BUFFER_XY_MAXIMUM ) {
        rear -= BUFFER_XY_MAXIMUM;
    }

    // add item to buffer

    bufferX[rear] = positionX;
    bufferY[rear] = positionY;

    // increment number of items
    if( itemCount < BUFFER_XY_MAXIMUM ) {
        itemCount++;
    }else{
        // no room for new items so drop oldest item
        head++;
        if( head >= BUFFER_XY_MAXIMUM ) {
            head = 0;
        }
    }
}

void getFrontHistPositionBaseEstXY(float *posbaseX, float *posbaseY)
{

	if(itemCount==0){
		*posbaseX=bufferX[head];
		*posbaseY=bufferY[head];
	return;
	}

	*posbaseX=bufferX[head];
	*posbaseY=bufferY[head];
	head++;

	if(head>=BUFFER_XY_MAXIMUM)
	{
		head=0;
	}
	itemCount--;
}



bool isPositionBaseXYQueueIsFull(void)
{
	return itemCount>=BUFFER_XY_MAXIMUM;
}



void checkPosition()
{

	switch(localisationType)
	{
		case LOC_UWB:
		// for uwb

		inputX = 1.0f*(float)posX;///10;	//centimeter
		inputY = -1.0f*(float)posY;///10;
		inputZ = 1.0f*(float)posZ;//10;
		break;

		case LOC_WHYCON:
		// for whycon
		inputX = -1.0f*(float)posY;///10;	//centimeter
		inputY = -1.0f*(float)posX;///10;
		break;

		case LOC_VICON:
		// for vicon
		inputX = 1.0f*(float)posX;///10;	//centimeter
		inputY = 1.0f*(float)posY;///10;
		inputZ = 1.0f*(float)posZ;//10;
		break;

		default:
		break;

	}

	//Discard the first 10 readings
	if(first_read <= 10) {
		setPos(inputX, inputY);
		first_read++;
	}


	if (isPositionBaseXYQueueIsFull())
		getFrontHistPositionBaseEstXY(&hist_position_baseX, &hist_position_baseY);
	else{
		hist_position_baseX = position_base[0];
		hist_position_baseY = position_base[1];
	}

	position_error[0] = inputX - (hist_position_baseX + position_correction[0]);
	position_error[1] = inputY - (hist_position_baseY + position_correction[1]);

}





void PosXYEstimate(uint32_t currentTime)
{
    static uint32_t previousTime = 0, previousReadTime = 0;
    float dt = ((float)currentTime - (float)previousTime) / 1000000.0f;       //sec

    if ((currentTime - previousTime) < 10*MILLI_SEC)	//10ms
        return;

    previousTime = currentTime;

	if(new_position){
		checkPosition();
		new_position = false;
		}
		else if((currentTime-previousReadTime > 3*SEC) && (!ARMING_FLAG(ARMED)))//Didn't receive data for 2 seconds reset integrator
		{
//			resetPosition(); Later uncomment for faster convergence
		}


    for (i = 0; i < 2; i++) {
        if (accSumCountXYZ) {
            accel_EF_prev[i] = accel_EF[i];
            accel_EF[i] = (float) accSum[i] / (float) accSumCountXYZ;
        } else {
        	accel_EF[i] = 0;
        }

        accel_EF[i] = accel_EF[i] * accVelScale;
        accel_EF[i] = constrainf(accel_EF[i], -800, 800);



        if (i == 1)
            imuResetAccelerationSum(0);

        accel_EF_correction[i] += position_error[i] * k3 * dt;
        est_velocity[i] += position_error[i] * k2 * dt;
        position_correction[i] += position_error[i] * k1 * dt;

        //Estimation
        accel_EF[i] += accel_EF_correction[i];
        velocity_increase[i] = (accel_EF[i]) * dt; // acc * dt
        position_base[i] += (est_velocity[i] + velocity_increase[i] * 0.5f)
                * dt; //S = S0 + u + (1/2) at^2

        //Updated pos and vel
        est_position[i] = position_base[i] + position_correction[i];
        est_velocity[i] += velocity_increase[i]; //v = u + at
    }



    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
    hist_xy_counter++;
    if (hist_xy_counter >= INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS) {

        hist_xy_counter = 0;
        addHistPositionBaseEstXY(position_base[0], position_base[1]);

    }

    PositionX = (int16_t) est_position[0];
    PositionY = (int16_t) est_position[1];
    VelocityX = (int16_t) est_velocity[0];
    VelocityY = (int16_t) est_velocity[1];

}

void updatePosGains(void)
{
    //time_constant_xy=(float)pidProfile->I8[PIDNAVR]/10;
    if (time_constant_xy == 0.0f) {
        k1 = k2 = k3 = 0.0f;
    } else {
        k1 = 3.0f / time_constant_xy;
        k2 = 3.0f / (time_constant_xy * time_constant_xy);
        k3 = 1.0f / (time_constant_xy * time_constant_xy * time_constant_xy);
    }
}

void setPos(float newX, float newY)
{
    position_base[0] = newX;
    position_base[1] = newY;
    position_correction[0] = position_correction[1] = 0;
	est_position[0] = newX;
	est_position[1] = newY;

//    clearQueue();

//    hist_xy_counter = 0;
//    addHistPositionBaseEstXY(position_base[0], position_base[1]);
    imuResetAccelerationSum(0);
}

void configurePosHold2(pidProfile_t *initialPidProfile)
{
    pidProfile = initialPidProfile;
}
