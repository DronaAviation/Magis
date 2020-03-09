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
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"


#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "io/beeper.h"
#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/acceleration.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "API/Utils.h"

int16_t accADC[XYZ_AXIS_COUNT];
float accel_offset[XYZ_AXIS_COUNT]={0,0,0};
float accel_scale[XYZ_AXIS_COUNT]={1,1,1};

//float cal_samples[6][3];

float debug_beta[6];




acc_t acc;                       // acc access functions
sensor_align_e accAlign = (sensor_align_e) 0;
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.

uint16_t calibratingA = 0; // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationArmed;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

static flightDynamicsTrims_t *accelerationTrims;
static flightAccelCalData_T *flightAccelCalData;

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingA = calibrationCyclesRequired;
}

bool isAccelerationCalibrationComplete(void)
{
    return calibratingA == 0;
}

bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    rollAndPitchTrims->values.roll = 0;
    rollAndPitchTrims->values.pitch = 0;
}

void performAcclerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];
    uint8_t axis;

    for (axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle())
            a[axis] = 0;

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        accADC[axis] = 0;
        accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc_1G;

        resetRollAndPitchTrims(rollAndPitchTrims);

        saveConfigAndNotify();

        accZoffsetCalCycle = 100;

    }

    calibratingA--;
}

void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    uint8_t axis;
    static int32_t b[3];
    static int16_t accZero_saved[3] = { 0, 0, 0 };
    static rollAndPitchTrims_t angleTrim_saved = { { 0, 0 } };

    // Saving old zeropoints before measurement
    if (InflightcalibratingA == 50) {
        accZero_saved[X] = accelerationTrims->raw[X];
        accZero_saved[Y] = accelerationTrims->raw[Y];
        accZero_saved[Z] = accelerationTrims->raw[Z];
        angleTrim_saved.values.roll = rollAndPitchTrims->values.roll;
        angleTrim_saved.values.pitch = rollAndPitchTrims->values.pitch;
    }
    if (InflightcalibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
                b[axis] = 0;
            // Sum up 50 readings
            b[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            accelerationTrims->raw[axis] = 0;
        }
        // all values are measured
        if (InflightcalibratingA == 1) {
            AccInflightCalibrationActive = false;
            AccInflightCalibrationMeasurementDone = true;
            beeper(BEEPER_ACC_CALIBRATION); // indicate end of calibration
            // recover saved values to maintain current flight behaviour until new values are transferred
            accelerationTrims->raw[X] = accZero_saved[X];
            accelerationTrims->raw[Y] = accZero_saved[Y];
            accelerationTrims->raw[Z] = accZero_saved[Z];
            rollAndPitchTrims->values.roll = angleTrim_saved.values.roll;
            rollAndPitchTrims->values.pitch = angleTrim_saved.values.pitch;
        }
        InflightcalibratingA--;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm) {      // the aircraft is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = false;
        accelerationTrims->raw[X] = b[X] / 50;
        accelerationTrims->raw[Y] = b[Y] / 50;
        accelerationTrims->raw[Z] = b[Z] / 50 - acc_1G;    // for nunchuck 200=1G

        resetRollAndPitchTrims(rollAndPitchTrims);

        saveConfigAndNotify();
    }
}

void applyAccelerationTrims(flightDynamicsTrims_t *accelerationTrims)
{
    accADC[X] -= accelerationTrims->raw[X];
    accADC[Y] -= accelerationTrims->raw[Y];
    accADC[Z] -= accelerationTrims->raw[Z];
}

void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims)
{
    if (!acc.read(accADC)) {
        return;
    }
    alignSensors(accADC, accADC, accAlign);


#ifdef SIX_POINT_CALIBRATION

    if (!isAccelerationCalibrationComplete()) {
        calibrate_accel(1,1);
    }


    accADC[0]*=flightAccelCalData->accel_scale[0];
    accADC[1]*=flightAccelCalData->accel_scale[1];
    accADC[2]*=flightAccelCalData->accel_scale[2];


    accADC[0]-=flightAccelCalData->accel_offset[0];
    accADC[1]-=flightAccelCalData->accel_offset[1];
    accADC[2]-=flightAccelCalData->accel_offset[2];

#else


    if (!isAccelerationCalibrationComplete()) {
        performAcclerationCalibration(rollAndPitchTrims);

    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(rollAndPitchTrims);
    }

    applyAccelerationTrims(accelerationTrims);


#endif


}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationTrims = accelerationTrimsToUse;
}


void setAccelerationCalibration(flightAccelCalData_T *accelerationCalDataToUse)
{
	flightAccelCalData = accelerationCalDataToUse;
}

flightAccelCalData_T* getAccelerationCalibration()
{
	return flightAccelCalData;
}




/***************************************************************************************************/



void calibrate_update_matrices(float dS[6], float JS[6][6],
                                    float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];

    for( j=0; j<3; j++ ) {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }

    for( j=0; j<6; j++ ) {
        dS[j] += jacobian[j]*residual;
        for( k=0; k<6; k++ ) {
            JS[j][k] += jacobian[j]*jacobian[k];
        }
    }
}


// _calibrate_reset_matrices - clears matrices
void calibrate_reset_matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ ) {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ ) {
            JS[j][k] = 0.0f;
        }
    }
}

void calibrate_find_delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;

    //make upper triangular
    for( i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for( i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;

        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }

    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}




// _calibrate_model - perform low level accel calibration
// accel_sample are accelerometer samples collected in 6 different positions
// accel_offsets are output from the calibration routine
// accel_scale are output from the calibration routine
// returns true if successful
bool calibrate_accel(float samples[6][3],float accel_offsets[3], float accel_scale[3] )
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
    float data[3];
    float beta[6];
    float delta[6];
    float ds[6];
    float JS[6][6];
    bool success = true;


    float test;

    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_CSS;

    while( num_iterations < 20 && change > eps ) {
        num_iterations++;

        calibrate_reset_matrices(ds, JS);

        for( i=0; i<6; i++ ) {
            data[0] = samples[i][0];
            data[1] = samples[i][1];
            data[2] = samples[i][2];

            calibrate_update_matrices(ds, JS, beta, data);
        }

        calibrate_find_delta(ds, JS, delta);

        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);

        for( i=0; i<6; i++ ) {
            beta[i] -= delta[i];
        }
    }


    for( i=0; i<6; i++ ) {

    	debug_beta[i]=beta[i];
    }


    // copy results out
    accel_scale[0] = beta[3] * GRAVITY_CSS;
    accel_scale[1] = beta[4] * GRAVITY_CSS;
    accel_scale[2] = beta[5] * GRAVITY_CSS;
    accel_offsets[0] = beta[0] * accel_scale[0];
    accel_offsets[1] = beta[1] * accel_scale[1];
    accel_offsets[2] = beta[2] * accel_scale[2];

    // sanity check scale
    if( isnan(accel_scale[0]) || fabsf(accel_scale[0]-1.0f) > 0.1f || fabsf(accel_scale[1]-1.0f) > 0.1f || fabsf(accel_scale[2]-1.0f) > 0.1f ) {
        success = false;
    }
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
//    if(isnan(accel_offsets[0]) || fabsf(accel_offsets[0]) > 3.5f || fabsf(accel_offsets[1]) > 3.5f || fabsf(accel_offsets[2]) > 3.5f ) {
//        success = false;
//    }








    // return success or failure
    return success;
}
// calibrate_accel - perform accelerometer calibration including providing user
// instructions and feedback Gauss-Newton accel calibration routines borrowed
// from Rolfe Schmidt blog post describing the method:
// http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
// original sketch available at
// http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
void calibrate_accel(float trim_roll, float trim_pitch)
{

	float samples[6][3];
    float new_offsets[3];
    float new_scaling[3];
    float orig_offset[3];
    float orig_scale[3];
    uint8_t num_ok = 0;
    bool isReadFailed=false;


//    orig_offset[0]=accel_offset[0];
//    orig_offset[1]=accel_offset[1];
//    orig_offset[2]=accel_offset[2];
//
//    orig_scale[0]=accel_scale[0];
//    orig_scale[1]=accel_scale[1];
//    orig_scale[2]=accel_scale[2];

//    accel_offset[0]=0;
//    accel_offset[1]=0;
//    accel_offset[2]=0;
//
//    accel_scale[0]=1;
//    accel_scale[1]=1;
//    accel_scale[2]=1;



    LED.set(RED, OFF);
    LED.set(GREEN, OFF);
    LED.set(BLUE, OFF);

    delay(3000);




    // capture data from 6 positions
    for (uint8_t i=0; i<6; i++) {

    	if(isReadFailed)
    		break;

        // display message to user
        switch ( i ) {
            case 0:

                LED.set(RED, ON);
                LED.set(GREEN, OFF);
                LED.set(BLUE, OFF);

                break;
            case 1:

                LED.set(RED, OFF);
                LED.set(GREEN, ON);
                LED.set(BLUE, OFF);


                break;
            case 2:

                LED.set(RED, OFF);
                LED.set(GREEN, OFF);
                LED.set(BLUE, ON);


                break;
            case 3:

                LED.set(RED, ON);
                LED.set(GREEN, ON);
                LED.set(BLUE, OFF);


                break;
            case 4:

                LED.set(RED, ON);
                LED.set(GREEN, OFF);
                LED.set(BLUE, ON);


                break;
            default:    // default added to avoid compiler warning
            case 5:
                LED.set(RED, OFF);
                LED.set(GREEN, ON);
                LED.set(BLUE, ON);


                break;
        }


        delay(6000);

        // clear out any existing samples from ins
        if (!acc.read(accADC)) {
        	isReadFailed=true;
        	break;
        }


        // average 32 samples

            samples[i][0] = 0;
            samples[i][1] = 0;
            samples[i][2] = 0;

        uint8_t num_samples = 0;
      while (num_samples < 32) {

          delay(200);

            // read samples from ins
            if (!acc.read(accADC)) {
            	isReadFailed=true;
            	break;
            }
            alignSensors(accADC, accADC, accAlign);
            // capture sample

            samples[i][0]  += accADC[0];
            samples[i][1]  += accADC[1];
            samples[i][2]  += accADC[2];


            delay(10);

            num_samples++;
        }
        samples[i][0] /= num_samples;
        samples[i][1] /= num_samples;
        samples[i][2] /= num_samples;
    }

    // run the calibration routine

//    for (uint8_t i=0; i<6; i++) {
//
//        cal_samples[i][0]  = samples[i][0];
//        cal_samples[i][1]  = samples[i][1];
//        cal_samples[i][2]  = samples[i][2];
//
//    }
//


    if(!isReadFailed){

    bool success = calibrate_accel(samples, new_offsets, new_scaling);

//	accel_offset[0]=new_offsets[0];
//	accel_offset[1]=new_offsets[1];
//	accel_offset[2]=new_offsets[2];
//
//
//	accel_scale[0]=new_scaling[0];
//	accel_scale[1]=new_scaling[1];
//	accel_scale[2]=new_scaling[2];


    if (success) {
//        interact->printf_P(PSTR("Calibration successful\n"));
//
//        for (uint8_t k=0; k<num_accels; k++) {
//            // set and save calibration
//            _accel_offset[k].set(new_offsets[k]);
//            _accel_scale[k].set(new_scaling[k]);
//        }
//        _save_parameters();
//
//        // calculate the trims as well from primary accels and pass back to caller
//        _calculate_trim(samples[0][0], trim_roll, trim_pitch);

    	flightAccelCalData->accel_offset[0]=new_offsets[0];
    	flightAccelCalData->accel_offset[1]=new_offsets[1];
    	flightAccelCalData->accel_offset[2]=new_offsets[2];


    	flightAccelCalData->accel_scale[0]=new_scaling[0];
    	flightAccelCalData->accel_scale[1]=new_scaling[1];
    	flightAccelCalData->accel_scale[2]=new_scaling[2];

        LED.set(RED, ON);
        LED.set(GREEN, ON);
        LED.set(BLUE, ON);

        saveConfigAndNotify();

        delay(4000);

        calibratingA=0;
    	return;

    }else {

//    	accel_scale[0]=new_scaling[0];
//    	accel_scale[1]=new_scaling[1];
//    	accel_scale[2]=new_scaling[2];


	}


    }


    LED.set(RED, TOGGLE);
    LED.set(GREEN, OFF);
    LED.set(BLUE, OFF);

    delay(4000);


    calibratingA=0;

}





