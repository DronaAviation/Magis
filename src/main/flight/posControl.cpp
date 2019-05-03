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

#include "../API/API-Utils.h"
#include "io/rc_controls.h"


#define corr_scale 512/100
#define corr_scale2 1096/100

#define TOLERANCE_XY 10
#define CONTROL_ENDPOINT 200

uint8_t kp_posx;
uint8_t kp_posy;
uint8_t kp_velx;
uint8_t kp_vely;
uint8_t ki_velx;
uint8_t ki_vely;
uint8_t kd_velx;
uint8_t kd_vely;

uint8_t _new_kpx;
uint8_t _new_kpy;
uint8_t _new_kdx;
uint8_t _new_kdy;
uint8_t _new_kix;
uint8_t _new_kiy;

int32_t AltHoldfromPos = 100;
int16_t PID_x = 0;
int16_t PID_y = 0;
int16_t _new_IoutX = 0;
int16_t _new_IoutY = 0;
int16_t IoutX = 0;
int16_t IoutY = 0;
int16_t VdesiredX = 0;
int16_t VdesiredY = 0;
uint8_t HeightAchieved = 0;
static pidProfile_t *pidProfile; //PS2

int32_t debugPosCtr = 0;
int32_t debugPosCtr_1 = 0;
int32_t debugPosCtr_2 = 0;

void configurePosHold(pidProfile_t *initialPidProfile) //PS2
{
    pidProfile = initialPidProfile;
}

void VelocityController(int16_t VdesiredX, int16_t VdesiredY)
{
    kp_velx = kp_vely = pidProfile->P8[PIDPOSR];
    ki_velx = ki_vely = pidProfile->I8[PIDPOSR];
    kd_velx = kd_vely = pidProfile->D8[PIDPOSR];

    int16_t errorVelX, errorVelY;
    int16_t PoutX, PoutY, DoutX, DoutY;

    errorVelX = VdesiredX - VelocityX;
    errorVelY = VdesiredY - VelocityY;

    PoutX = (kp_velx * errorVelX) / 10;
    PoutY = (kp_vely * errorVelY) / 10;

    debugPosCtr = PoutX;

    if (ki_velx && ARMING_FLAG(ARMED) && isLocalisationOn) {
        IoutX += (ki_velx * errorVelX) / 10;
        IoutY += (ki_vely * errorVelY) / 10;
        IoutX = constrain(IoutX, -20000, 20000);
        IoutY = constrain(IoutY, -20000, 20000);
    } else { //Take out effects of integral if ki is zero
        IoutX = 0;
        IoutY = 0;
    }

    debugPosCtr_1 = IoutX;

    DoutX = (kd_velx * (accel_EF[0] + accel_EF_prev[0])) / 10;
    DoutY = (kd_vely * (accel_EF[1] + accel_EF_prev[1])) / 10;

    debugPosCtr_2 = DoutX;

    switch (localisationType) {

    case LOC_UWB:

        PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
        PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

        break;

    case LOC_WHYCON:

        PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
        PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

        break;

    case LOC_VICON:

        break;

    default:

        break;

    }

    //Debug PS2
    //PS2 Debug variables
    /* if(pidProfile->P8[PIDNAVR]==9) //Desired States
     {
     print_posvariable3 = PoutY*corr_scale; //10*VdesiredX*corr_scale;//ax
     print_posvariable4 = IoutY*corr_scale;//10*VelocityX*corr_scale;//ay
     print_posvariable6 = DoutY*corr_scale;
     print_posvariable1 = VdesiredY*corr_scale2;//mx
     print_posvariable2 = VelocityY*corr_scale2;//my
     print_posvariable5 = rcCommand[ROLL]*corr_scale2;
     } */

}

void PosVelController(int16_t desiredX, int16_t desiredY, int16_t desiredVel)
{

    kp_posx = kp_posy = pidProfile->P8[PIDPOS];

    int16_t errorX, errorY;

    errorX = desiredX - PositionX;
    errorY = desiredY - PositionY;

    VdesiredX = constrain((kp_posx * errorX) / 100, -desiredVel, desiredVel);
    VdesiredY = constrain((kp_posy * errorY) / 100, -desiredVel, desiredVel);

    VelocityController(VdesiredX, VdesiredY);

}

bool PositionController(int16_t desiredX, int16_t desiredY, int16_t desiredZ)
{
    bool Status = false;
    //setdesiredHeight(desiredZ);

    PosVelController(desiredX, desiredY, 50);
    //SimpleController(desiredX, desiredY, 50);
    if ((abs(desiredX - PositionX) < TOLERANCE_XY)
            && (abs(desiredY - PositionY) < TOLERANCE_XY)
            && (HeightAchieved == 1)) {
        if (isLocalisationOn) {	//&&ARMING_FLAG(ARMED))
            Status = true;
        } else {
            Status = false;
        }
    }

    /* if(pidProfile->P8[PIDNAVR]==8) //Desired States
     {
     print_posvariable3 = desiredX*corr_scale;//ax
     print_posvariable4 = desiredY*corr_scale;//ay
     print_posvariable6 = 100*HeightAchieved*corr_scale;//az
     print_posvariable1 = (IoutX*corr_scale2)/500;//mx
     print_posvariable2 = (IoutY*corr_scale2)/500;//my
     print_posvariable5 = 100*corr_scale2;
     } */

    return Status;
}

void setdesiredHeight(int32_t desiredZ)
{
    AltHoldfromPos = desiredZ;
}

int32_t getdesiredHeight(void)
{
    return AltHoldfromPos;
}

int16_t getrcDataRoll(void)
{
    return PID_y;
}

int16_t getrcDataPitch(void)
{
    return PID_x;
}

int16_t getDesiredVelocityX()
{
    return VdesiredX;

}
int16_t getDesiredVelocityY()
{

    return VdesiredY;
}

void resetPosIntegral(void)
{	//To Eliminate windup
    _new_IoutX = 0;
    _new_IoutY = 0;
    IoutX = 0;
    IoutY = 0;

}

bool SimpleController(int16_t desiredX, int16_t desiredY, int16_t desiredZ)
{
    int16_t errorX, errorY;
    bool Status = false;

    //setdesiredHeight(desiredZ); //Set Desired Height

    _new_kpx = _new_kpy = pidProfile->P8[PIDPOSR];
    _new_kix = _new_kiy = pidProfile->I8[PIDPOSR];
    _new_kdx = _new_kdy = pidProfile->D8[PIDPOSR];

    errorX = constrain(desiredX - PositionX, -50, 50);
    errorY = constrain(desiredY - PositionY, -50, 50);
    //errorX = desiredX-PositionX;
    //errorY = desiredY-PositionY;

    if (_new_kix && ARMING_FLAG(ARMED) && isLocalisationOn) {
        _new_IoutX += (_new_kix * (errorX)) / 10;
        _new_IoutX = constrain(_new_IoutX, -15000, 15000);

        _new_IoutY += (_new_kiy * (errorY)) / 10;
        _new_IoutY = constrain(_new_IoutY, -15000, 15000);

    } else {
        _new_IoutX = 0;
        _new_IoutY = 0;
    }

    PID_x = (_new_kpx * errorX) / 10 - constrain(_new_kdx * VelocityX, -120, 120) + _new_IoutX / 500;
    PID_y = -((_new_kpy * errorY) / 10 - constrain(_new_kdy * VelocityY, -120, 120) + _new_IoutY / 500);
    PID_x = constrain(PID_x, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
    PID_y = constrain(PID_y, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

    //Has the command been executed??
    if ((abs(errorX) < TOLERANCE_XY) && (abs(errorY) < TOLERANCE_XY) && (HeightAchieved == 1)) {
        if (isLocalisationOn) {	//&&ARMING_FLAG(ARMED))
            Status = true;
        } else {
            Status = false;
        }
    }

    //Debug
    /* if(pidProfile->P8[PIDNAVR]==10)
     {
     print_posvariable3 = 12*corr_scale;
     print_posvariable4 = 13*corr_scale;
     print_posvariable1 = 10*corr_scale2;//sin_approx((float)(pidProfile->D8[PIDNAVR]*10)*RAD)*corr_scale2;
     print_posvariable2 = 15*corr_scale2;
     }else if(pidProfile->P8[PIDNAVR]==0) //State variables
     {
     print_posvariable3 = PositionX*corr_scale;//ax
     print_posvariable4 = PositionY*corr_scale;//ay
     print_posvariable1 = VelocityX*corr_scale2;//mx
     print_posvariable2 = VelocityY*corr_scale2;//my
     }else if(pidProfile->P8[PIDNAVR]==3) //Desired States
     {
     uint8_t scale = (acc_1G > 1024) ? 8 : 1;
     print_posvariable3 = accSmooth[0]/scale;//*corr_scale;//ax
     print_posvariable4 = accSmooth[1]/scale;//ay
     print_posvariable6 = (_new_IoutY*corr_scale)/500;//aZ
     print_posvariable1 = (accel[0]*100)/980*corr_scale2;//mx
     print_posvariable2 = (accel[1]*100)/980*corr_scale2;//my
     print_posvariable5 = 0*corr_scale2;//my
     }else if(pidProfile->P8[PIDNAVR]==4) //Desired States
     {
     print_posvariable3 = VelocityX*corr_scale;//ax
     print_posvariable4 = VelocityY*corr_scale;//ay
     print_posvariable1 = WhyconVx*corr_scale2;//mx
     print_posvariable2 = WhyconVy*corr_scale2;//my
     }
     else if(pidProfile->P8[PIDNAVR]==5) //Desired States
     {
     print_posvariable3 = (_new_kpy*errorY)*corr_scale;//ax
     print_posvariable4 = (_new_kdy*VelocityY)*corr_scale;//ay
     print_posvariable6 = (_new_IoutY/500)*corr_scale;//az
     print_posvariable1 = _new_IoutX*corr_scale2/500;//mx
     print_posvariable2 = _new_IoutY*corr_scale2/500;//my
     } */

    return Status;
}
