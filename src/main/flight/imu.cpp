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
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

// Inertial Measurement Unit (IMU)
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/maths.h"

#include "platform.h"
#include "debug.h"

#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"
#include "drivers/gpio.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "io/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "API/API-Utils.h"
#include "config/runtime_config.h"
#include "mw.h"

int16_t accSmooth[XYZ_AXIS_COUNT];
int32_t accSum[XYZ_AXIS_COUNT];
int32_t accSumXYZ[XYZ_AXIS_COUNT];

uint32_t accTimeSum = 0;        // keep track for integration of acc
int accSumCount = 0,accSumCountXYZ=0;
float accVelScale;

int16_t smallAngle = 0;

float throttleAngleScale;
float fc_acc;

//float magneticDeclination = 0.0f;       // calculated at startup from config
float gyroScaleRad;

rollAndPitchInclination_t inclination = { { 0, 0 } }; // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
rollAndPitchInclination_t inclination_generalised = { { 0, 0 } }; // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
float anglerad[ANGLE_INDEX_COUNT] = { 0.0f, 0.0f };    // absolute angle inclination in radians

static imuRuntimeConfig_t *imuRuntimeConfig;
static pidProfile_t *pidProfile;
static accDeadband_t *accDeadband;

int32_t netAccMagnitude = 0;
int32_t accZoffsetCalCycle = 0;

float tempRotation[3][3];

float tempMat[3][3];
float tempMat1[3][3];


//int32_t imuDebug=0;
//int32_t imuDebug1=0;


#ifdef DCM
/* variable declaration for DCM */


// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT 20

#define RP_P_MIN   0.05f        // minimum value for RP_P parameter
#define YAW_P_MIN  0.05f        // minimum value for AHRS_YAW_P parameter

// primary representation of attitude of board used for all inertial calculations
float dcm_matrix[3][3]={{1,0,0},{0,1,0},{0,0,1}};

// primary representation of attitude of flight vehicle body
float body_dcm_matrix[3][3]={0};

float omega_P[3]={0.0f};                          // accel Omega proportional correction
float omega_yaw_P[3]={0.0f};                      // proportional yaw correction
float omega_I[3]={0.0f};                          // Omega Integrator correction
float omega_I_sum[3]={0.0f};
float omega_I_sum_time=0;
float omega[3]={0.0f};

float renorm_val_sum;
uint16_t renorm_val_count;
float error_yaw_sum;
uint16_t error_yaw_count;

float accel_ef[3];
float ra_sum[3]={0};
float ra_deltaT=0;

float kp= 0.2f;
float ki= 0.0087f;
float kp_yaw=0.2f;
float ki_yaw=0.01f;

// time in microseconds of last compass update
 uint32_t compass_last_update;


// TODO have to move to appropriate file
float gyro_drift_limit= radians(0.5/60);    // 0.5 degrees/second/minute

float temp_mul_transpose[3];
float temp_mul_xy[2];
float temp_mul_xyz[3];
float temp_mul[2];


#endif

void imuConfigure(imuRuntimeConfig_t *initialImuRuntimeConfig, pidProfile_t *initialPidProfile, accDeadband_t *initialAccDeadband, float accz_lpf_cutoff, uint16_t throttle_correction_angle)
{
    imuRuntimeConfig = initialImuRuntimeConfig;
    pidProfile = initialPidProfile;
    accDeadband = initialAccDeadband;
    fc_acc = calculateAccZLowPassFilterRCTimeConstant(accz_lpf_cutoff);
    throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);
}

void imuInit(void)
{

    #ifdef DCM
    smallAngle=imuRuntimeConfig->small_angle;
    #else
    smallAngle = lrintf(acc_1G * cos_approx(degreesToRadians(imuRuntimeConfig->small_angle)));
    #endif

    accVelScale = 9.80665f / acc_1G * 100.0f;
    gyroScaleRad = gyro.scale * (M_PIf / 180.0f) * 0.000001f;
}

float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

/*
 * Calculate RC time constant used in the accZ lpf.
 */
float calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff)
{
    return 0.5f / (M_PIf * accz_lpf_cutoff);
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

t_fp_vector EstG;

void imuResetAccelerationSum(int i)
{
	if(i == 0){
			accSum[0] = 0;
			accSum[1] = 0;


// placement to be decided ; need for height cal from opticflow
			accSumXYZ[0] = 0;
			accSumXYZ[1] = 0;
			accSumXYZ[2] = 0;
			accSumCountXYZ = 0;
		}
	    else{
			accSum[2] = 0;
			accSumCount = 0;
		}
	    accTimeSum = 0;
}

// rotate acc into Earth frame and calculate acceleration in it
void imuCalculateAcceleration(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    float *tempVector;
   // static float accz_smooth = 0;
   // float dT;
    fp_angles_t rpy;
    t_fp_vector accel_ned;
    float accel_hbf[3];

    // deltaT is measured in us ticks
   // dT = (float) deltaT * 1e-6f;

    // the accel values have to be rotated into the earth frame
    rpy.angles.roll = -(float) anglerad[AI_ROLL];
    rpy.angles.pitch = -(float) anglerad[AI_PITCH];
    rpy.angles.yaw = (float)(heading-headFreeModeHold) * RAD;
   // rpy.angles.yaw = -(float)heading * RAD;
    accel_ned.V.X = accSmooth[0];
    accel_ned.V.Y = accSmooth[1];
    accel_ned.V.Z = accSmooth[2];


    accel_hbf[0] = accSmooth[0];
    accel_hbf[1] = accSmooth[1];
    accel_hbf[2] = accSmooth[2];

    rotateV(&accel_ned.V, &rpy);

    if (imuRuntimeConfig->acc_unarmedcal == 1) {
        if (!ARMING_FLAG(ARMED) && accZoffsetCalCycle) {
            accZoffset -= accZoffset / 8;
            accZoffset += accel_ned.V.Z;
            accZoffsetCalCycle--;

        }
        accel_ned.V.Z -= accZoffset / 8;  // compensate for gravitation on z-axis

        //netAccMagnitude=accel_ned.V.Z;

    } else
        accel_ned.V.Z -= acc_1G;

    // accz_smooth = accz_smooth + (dT / (fc_acc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence
    accSum[X] += applyDeadband(lrintf(accel_ned.V.X), accDeadband->xy);
    accSum[Y] += applyDeadband(lrintf(accel_ned.V.Y), accDeadband->xy);
    // accSum[Z] += applyDeadband(lrintf(accz_smooth), accDeadband->z);
    accSum[Z] += applyDeadband(lrintf(accel_ned.V.Z), accDeadband->z);

    tempVector=dcmBodyToEarth3D(accel_hbf);

    accSumXYZ[X]+=applyDeadband( tempVector[X], accDeadband->xy);
    accSumXYZ[Y]+=applyDeadband( tempVector[Y], accDeadband->xy);
    accSumXYZ[Z]+=applyDeadband( tempVector[Z], accDeadband->z);


    // sum up Values for later integration to get velocity and distance
    accTimeSum += deltaT;
    accSumCount++;
    accSumCountXYZ++;
}

/*
 * Baseflight calculation by Luggi09 originates from arducopter
 * ============================================================
 * This function rotates magnetic vector to cancel actual yaw and
 * pitch of craft. Then it computes it's direction in X/Y plane.
 * This value is returned as compass heading, value is 0-360 degrees.
 *
 * Note that Earth's magnetic field is not parallel with ground unless
 * you are near equator. Its inclination is considerable, >60 degrees
 * towards ground in most of Europe.
 *
 * First we consider it in 2D:
 *
 * An example, the vector <1, 1> would be turned into the heading
 * 45 degrees, representing it's angle clockwise from north.
 *
 *      ***************** *
 *      *       |   <1,1> *
 *      *       |  /      *
 *      *       | /       *
 *      *       |/        *
 *      *       *         *
 *      *                 *
 *      *                 *
 *      *                 *
 *      *                 *
 *      *******************
 *
 * //TODO: Add explanation for how it uses the Z dimension.
 */
int16_t imuCalculateHeading(t_fp_vector *vec)
{
    int16_t head;

    float cosineRoll = cos_approx(anglerad[AI_ROLL]);
    float sineRoll = sin_approx(anglerad[AI_ROLL]);
    float cosinePitch = cos_approx(anglerad[AI_PITCH]);
    float sinePitch = sin_approx(anglerad[AI_PITCH]);
    float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
    float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
    //TODO: Replace this comment with an explanation of why Yh and Xh can never simultanoeusly be zero,
    // or handle the case in which they are and (atan2f(0, 0) is undefined.

    float hd = (atan2f(Yh, Xh) * 1800.0f / M_PIf + magneticDeclination) / 10.0f;
    head = lrintf(hd);

    // Arctan returns a value in the range -180 to 180 degrees. We 'normalize' negative angles to be positive.
    if (head < 0)
        head += 360;

    return head;
}

static void imuCalculateEstimatedAttitude(void)
{
    int32_t axis;
    int32_t accMag = 0;
    static t_fp_vector EstM;
    static t_fp_vector EstN = { .A = { 1.0f, 0.0f, 0.0f } };
    static float accLPF[3];
    static uint32_t previousT;
    uint32_t currentT = micros();
    uint32_t deltaT;
    float scale;
    fp_angles_t deltaGyroAngle;
    deltaT = currentT - previousT;
    scale = deltaT * gyroScaleRad;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle.raw[axis] = gyroADC[axis] * scale;
        if (imuRuntimeConfig->acc_lpf_factor > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / imuRuntimeConfig->acc_lpf_factor)) + accADC[axis] * (1.0f / imuRuntimeConfig->acc_lpf_factor);
            accSmooth[axis] = (int16_t) accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }

        //netAccMagnitude=accSmooth[axis]  ;

        accMag += ((int32_t) accSmooth[axis] * (int32_t) accSmooth[axis]) / (int32_t) acc_1G;
    }
    accMag = accMag * 100 / (int32_t) acc_1G;

    netAccMagnitude = accMag;

    rotateV(&EstG.V, &deltaGyroAngle);





    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro

    float invGyroComplimentaryFilterFactor = (1.0f / (imuRuntimeConfig->gyro_cmpf_factor + 1.0f));

    if (72 < (uint16_t) accMag && (uint16_t) accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * imuRuntimeConfig->gyro_cmpf_factor + accSmooth[axis]) * invGyroComplimentaryFilterFactor;

    }

    if (EstG.A[Z] > smallAngle) {
        ENABLE_STATE(SMALL_ANGLE);
    } else {
        DISABLE_STATE(SMALL_ANGLE);
    }

    // Attitude of the estimated vector
    anglerad[AI_ROLL] = atan2f(EstG.V.Y, EstG.V.Z);
    anglerad[AI_PITCH] = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));
    inclination.values.rollDeciDegrees = lrintf(anglerad[AI_ROLL] * (1800.0f / M_PIf));
    inclination.values.pitchDeciDegrees = lrintf(anglerad[AI_PITCH] * (1800.0f / M_PIf));



    inclination_generalised.values.pitchDeciDegrees=  inclination.values.pitchDeciDegrees;

    if(!reverseReferenceFrame)
  	  inclination_generalised.values.rollDeciDegrees= inclination.values.rollDeciDegrees;
    else
    {
    	inclination_generalised.values.rollDeciDegrees= inclination.values.rollDeciDegrees+1800;
    	if(inclination_generalised.values.rollDeciDegrees>1800)
    		inclination_generalised.values.rollDeciDegrees=3600 - inclination_generalised.values.rollDeciDegrees;
    	else
    		inclination_generalised.values.rollDeciDegrees=-(inclination_generalised.values.rollDeciDegrees);
    }

    if (sensors(SENSOR_MAG)) {
        rotateV(&EstM.V, &deltaGyroAngle);
        // FIXME what does the _M_ mean?
        float invGyroComplimentaryFilter_M_Factor = (1.0f / (imuRuntimeConfig->gyro_cmpfm_factor + 1.0f));
        for (axis = 0; axis < 3; axis++) {
            EstM.A[axis] = (EstM.A[axis] * imuRuntimeConfig->gyro_cmpfm_factor + magADC[axis]) * invGyroComplimentaryFilter_M_Factor;
        }
        heading = imuCalculateHeading(&EstM);
    } else {
        rotateV(&EstN.V, &deltaGyroAngle);
        normalizeV(&EstN.V, &EstN.V);
        heading = imuCalculateHeading(&EstN);
    }

    imuCalculateAcceleration(deltaT); // rotate acc vector into earth frame
}

#ifdef DCM

void dcmFromEuler(float roll, float pitch, float yaw)
{

    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    dcm_matrix[0][0] = cp * cy;
    dcm_matrix[0][1] = (sr * sp * cy) - (cr * sy);
    dcm_matrix[0][2] = (cr * sp * cy) + (sr * sy);
    dcm_matrix[1][0] = cp * sy;
    dcm_matrix[1][1] = (sr * sp * sy) + (cr * cy);
    dcm_matrix[1][2] = (cr * sp * sy) - (sr * cy);
    dcm_matrix[2][0] = -sp;
    dcm_matrix[2][1] = sr * cp;
    dcm_matrix[2][2] = cr * cp;



}


void bodyDcmToEuler(float *roll, float *pitch, float *yaw)
{
    if (pitch != NULL) {
        *pitch = -safe_asin(body_dcm_matrix[2][0]);

    }
    if (roll != NULL) {
        *roll = atan2f(body_dcm_matrix[2][1], body_dcm_matrix[2][2]);


    }
    if (yaw != NULL) {
        *yaw = atan2f(body_dcm_matrix[1][0], body_dcm_matrix[0][0]);

    }
}


void rotateDCM(float *g)
{
    float temp_matrix[3][3];

    temp_matrix[0][0]=dcm_matrix[0][1]*g[2]-dcm_matrix[0][2]*g[1];
    temp_matrix[0][1]=dcm_matrix[0][2]*g[0]-dcm_matrix[0][0]*g[2];
    temp_matrix[0][2]=dcm_matrix[0][0]*g[1]-dcm_matrix[0][1]*g[0];
    temp_matrix[1][0]=dcm_matrix[1][1]*g[2]-dcm_matrix[1][2]*g[1];
    temp_matrix[1][1]=dcm_matrix[1][2]*g[0]-dcm_matrix[1][0]*g[2];
    temp_matrix[1][2]=dcm_matrix[1][0]*g[1]-dcm_matrix[1][1]*g[0];
    temp_matrix[2][0]=dcm_matrix[2][1]*g[2]-dcm_matrix[2][2]*g[1];
    temp_matrix[2][1]=dcm_matrix[2][2]*g[0]-dcm_matrix[2][0]*g[2];
    temp_matrix[2][2]=dcm_matrix[2][0]*g[1]-dcm_matrix[2][1]*g[0];


    dcm_matrix[0][0]+=temp_matrix[0][0];
    dcm_matrix[0][1]+=temp_matrix[0][1];
    dcm_matrix[0][2]+=temp_matrix[0][2];
    dcm_matrix[1][0]+=temp_matrix[1][0];
    dcm_matrix[1][1]+=temp_matrix[1][1];
    dcm_matrix[1][2]+=temp_matrix[1][2];
    dcm_matrix[2][0]+=temp_matrix[2][0];
    dcm_matrix[2][1]+=temp_matrix[2][1];
    dcm_matrix[2][2]+=temp_matrix[2][2];




}


float* dcmMulTranspose(float *vector){



    temp_mul_transpose[0]= dcm_matrix[0][0]*vector[0] + dcm_matrix[1][0]*vector[1] + dcm_matrix[2][0]*vector[2];
    temp_mul_transpose[1]= dcm_matrix[0][1]*vector[0] + dcm_matrix[1][1]*vector[1] + dcm_matrix[2][1]*vector[2];
    temp_mul_transpose[2]= dcm_matrix[0][2]*vector[0] + dcm_matrix[1][2]*vector[1] + dcm_matrix[2][2]*vector[2];

    return temp_mul_transpose;




}

// multiplication by a vector, extracting only the xy components
float * dcmMulXY(float* vector)
{
//
//    temp_mul_xy[0]= tempRotation[0][0]*vector[0] + tempRotation[0][1]*vector[1] + tempRotation[0][2]*vector[2];
//    temp_mul_xy[1]= tempRotation[1][0]*vector[0] + tempRotation[1][1]*vector[1] + tempRotation[1][2]*vector[2];
//
//    return temp_mul_xy;

    temp_mul_xy[0]= dcm_matrix[0][0]*vector[0] + dcm_matrix[0][1]*vector[1] + dcm_matrix[0][2]*vector[2];
    temp_mul_xy[1]= dcm_matrix[1][0]*vector[0] + dcm_matrix[1][1]*vector[1] + dcm_matrix[1][2]*vector[2];

    return temp_mul_xy;



}




float * dcmBodyToEarth3D(float* vector)
{

//    float roll= -radians(0);
//    float pitch= -radians(-30);
//    float yaw=0;


    float roll= anglerad[AI_ROLL];
    float pitch= anglerad[AI_PITCH];
    float yaw= 0;


    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    tempRotation[0][0] = cp * cy;
    tempRotation[0][1] = (sr * sp * cy) - (cr * sy);
    tempRotation[0][2] = (cr * sp * cy) + (sr * sy);
    tempRotation[1][0] = cp * sy;
    tempRotation[1][1] = (sr * sp * sy) + (cr * cy);
    tempRotation[1][2] = (cr * sp * sy) - (sr * cy);
    tempRotation[2][0] = -sp;
    tempRotation[2][1] = sr * cp;
    tempRotation[2][2] = cr * cp;


    tempMat1[0][0]=vector[0];
    tempMat1[0][1]=vector[1];
    tempMat1[0][2]=vector[2];

    temp_mul_xyz[0]= tempRotation[0][0]*vector[0] + tempRotation[0][1]*vector[1] + tempRotation[0][2]*vector[2];
    temp_mul_xyz[1]= tempRotation[1][0]*vector[0] + tempRotation[1][1]*vector[1] + tempRotation[1][2]*vector[2];
    temp_mul_xyz[2]= tempRotation[2][0]*vector[0] + tempRotation[2][1]*vector[1] + tempRotation[2][2]*vector[2];


    return temp_mul_xyz;
}


float * earthToBody2D(float* vector){



    float sy = sinf(anglerad[AI_YAW]);
    float cy = cosf(anglerad[AI_YAW]);

    temp_mul[0] = vector[0]*cy+vector[1]*sy;

    temp_mul[1] = -vector[0]*sy+vector[1]*cy;

    return temp_mul;
}




void matrix_update(float deltaT){


    float scale=0.0174532f/16.4f;

    //setting omega values from gyro data
    omega[0]=(float)gyroADC[0]*scale;
    omega[1]=(float)gyroADC[1]*scale;
    omega[2]=(float)gyroADC[2]*scale;

   // debugIMU=  omega[0];



//    debugIMU=omega[0]*1000;
//    debugIMU_1=omega[1]*1000;
//    debugIMU_2=omega[2]*1000;
//    debugIMU_3=omega_P[0]*1000;
//    debugIMU_4=omega_P[1]*1000;
//    debugIMU_5=omega_P[2]*1000;
//    debugIMU_6=omega_I[0]*100000;
//    debugIMU_7=omega_I[1]*100000;
//    debugIMU_8=omega_I[2]*100000;


    //Adding integrals
    omega[0]+= omega_I[0];
    omega[1]+= omega_I[1];
    omega[2]+= omega_I[2];

    //debugIMU= omega[0];

    float temp_g[3];

    temp_g[0]=(omega[0] + omega_P[0] + omega_yaw_P[0]) * deltaT;
    temp_g[1]=(omega[1] + omega_P[1] + omega_yaw_P[1]) * deltaT;
    temp_g[2]=(omega[2] + omega_P[2] + omega_yaw_P[2]) * deltaT;

//     temp_g[0]=(omega[0] + omega_yaw_P[0]) * deltaT;
//     temp_g[1]=(omega[1] + omega_yaw_P[1]) * deltaT;
//     temp_g[2]=(omega[2] + omega_yaw_P[2]) * deltaT;


//    debugIMU= omega_P[1]*1000;
//    debugIMU_1= omega[1]*1000;
//    debugIMU_2= (omega_P[1]+omega[1])*1000;

    rotateDCM(temp_g);




}


void reset(bool recover_eulers)
{
    // reset the integration terms
    omega_I[0]=0;
    omega_I[1]=0;
    omega_I[2]=0;

    omega_P[0]=0;
    omega_P[1]=0;
    omega_P[2]=0;

    omega_yaw_P[0]=0;
    omega_yaw_P[1]=0;
    omega_yaw_P[2]=0;

    omega[0]=0;
    omega[1]=0;
    omega[2]=0;

    //debugIMU++;



    // if the caller wants us to try to recover to the current
    // attitude then calculate the dcm matrix from the current
    // roll/pitch/yaw values
    if (recover_eulers && !isnan(anglerad[AI_ROLL]) && !isnan(anglerad[AI_ROLL]) && !isnan(anglerad[AI_YAW])) {
        dcmFromEuler(anglerad[AI_ROLL], anglerad[AI_PITCH], anglerad[AI_YAW]);
    } else {
   //  otherwise make it flat
        dcmFromEuler(0, 0, 0);
    }
}




bool renorm(float const *a, float *result)
{

    float renorm_val;

    renorm_val = 1.0f / sqrtf(sq(a[0])+sq(a[1])+sq(a[2]));


    // keep the average for reporting
   renorm_val_sum += renorm_val;
   renorm_val_count++;

   if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
       // this is larger than it should get - log it as a warning
       if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
           // we are getting values which are way out of
           // range, we will reset the matrix and hope we
           // can recover our attitude using drift
           // correction before we hit the ground!
           //Serial.printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
           //     renorm_val);


           return false;
       }
   }

   result[0] = a[0] * renorm_val;
   result[1] = a[1] * renorm_val;
   result[2] = a[2] * renorm_val;



   return true;

}

void normalize(void){

   float error;
   float t0[3];
   float t1[3];
   float t2[3];

   // dot product  eq. 18
   error= dcm_matrix[0][0]*dcm_matrix[1][0] + dcm_matrix[0][1]*dcm_matrix[1][1] + dcm_matrix[0][2]*dcm_matrix[1][2];


   // a eq. 19
   t0[0]= dcm_matrix[0][0] - (dcm_matrix[1][0] * (0.5f * error));
   t0[1]= dcm_matrix[0][1] - (dcm_matrix[1][1] * (0.5f * error));
   t0[2]= dcm_matrix[0][2] - (dcm_matrix[1][2] * (0.5f * error));

   // b eq. 19
   t1[0]= dcm_matrix[1][0] - (dcm_matrix[0][0] * (0.5f * error));
   t1[1]= dcm_matrix[1][1] - (dcm_matrix[0][1] * (0.5f * error));
   t1[2]= dcm_matrix[1][2] - (dcm_matrix[0][2] * (0.5f * error));


   // c= a x b  cross product  eq. 20
   t2[0]= t0[1]*t1[2] - t0[2]*t1[1];
   t2[1]= t0[2]*t1[0] - t0[0]*t1[2];
   t2[2]= t0[0]*t1[1] - t0[1]*t1[0];



   if (!renorm(t0, &dcm_matrix[0][0]) ||
       !renorm(t1, &dcm_matrix[1][0]) ||
       !renorm(t2, &dcm_matrix[2][0])) {
       // Our solution is blowing up and we will force back
       // to last euler angles
       //  _last_failure_ms = hal.scheduler->millis();


        reset(true);
   }





}


// the _P_gain raises the gain of the PI controller
// when we are spinning fast. See the fastRotations
// paper from Bill.
float pGain(float spin_rate)
{
    if (spin_rate < radians(50)) {
        return 1.0f;
    }
    if (spin_rate > radians(500)) {
        return 10.0f;
    }
    return spin_rate/radians(50);
}



void check_matrix(void)
{
  bool isNan=false;

  if(isnan(dcm_matrix[0][0])||isnan(dcm_matrix[0][1])||isnan(dcm_matrix[0][2]))
      isNan=true;

  if(isnan(dcm_matrix[1][0])||isnan(dcm_matrix[1][1])||isnan(dcm_matrix[1][2]))
      isNan=true;

  if(isnan(dcm_matrix[2][0])||isnan(dcm_matrix[2][1])||isnan(dcm_matrix[2][2]))
       isNan=true;


  if (isNan) {

      //   debugIMU=14;
         reset(true);
         return;
     }
     // some DCM matrix values can lead to an out of range error in
     // the pitch calculation via asin().  These NaN values can
     // feed back into the rest of the DCM matrix via the
     // error_course value.
     if (!(dcm_matrix[2][0] < 1.0f &&
           dcm_matrix[2][0] > -1.0f)) {
         // We have an invalid matrix. Force a normalisation.
         normalize();

         if (isNan ||
             fabsf(dcm_matrix[2][0]) > 10) {
             // normalisation didn't fix the problem! We're
             // in real trouble. All we can do is reset
             //Serial.printf("ERROR: DCM matrix error. _dcm_matrix.c.x=%f\n",
             //     _dcm_matrix.c.x);
         //    debugIMU=15;
             reset(true);
         }
     }




}



// calculate the euler angles and DCM matrix which will be used for high level
// navigation control. Apply trim such that a positive trim value results in a
// positive vehicle rotation about that axis (ie a negative offset)
void euler_angles(void)
{
    body_dcm_matrix[0][0] = dcm_matrix[0][0];
    body_dcm_matrix[0][1] = dcm_matrix[0][1];
    body_dcm_matrix[0][2] = dcm_matrix[0][2];
    body_dcm_matrix[1][0] = dcm_matrix[1][0];
    body_dcm_matrix[1][1] = dcm_matrix[1][1];
    body_dcm_matrix[1][2] = dcm_matrix[1][2];
    body_dcm_matrix[2][0] = dcm_matrix[2][0];
    body_dcm_matrix[2][1] = dcm_matrix[2][1];
    body_dcm_matrix[2][2] = dcm_matrix[2][2];


    //_body_dcm_matrix.rotateXYinv(_trim);

    bodyDcmToEuler(&anglerad[AI_ROLL], &anglerad[AI_PITCH], &anglerad[AI_YAW]);

   //  update_cd_values();
}

/*
  calculate a compass heading given the attitude from DCM and the mag vector
 */
float calculate_heading()
{
    float cos_pitch_sq = 1.0f-(dcm_matrix[2][0]*dcm_matrix[2][0]);

    // Tilt compensated magnetic field Y component:
    float headY = magADC[1] * dcm_matrix[2][2] -  magADC[2] * dcm_matrix[2][1];

    // Tilt compensated magnetic field X component:
    float headX =  magADC[0] * cos_pitch_sq - dcm_matrix[2][0] * ( magADC[1] * dcm_matrix[2][1] +  magADC[2] * dcm_matrix[2][2]);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float mag_heading = constrainf(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(magneticDeclination) > 0.0f )
    {
        mag_heading = mag_heading + magneticDeclination;
        if (mag_heading > M_PIf)    // Angle normalization (-180 deg, 180 deg)
            mag_heading -= (2.0f * M_PIf);
        else if (mag_heading < -M_PIf)
            mag_heading += (2.0f * M_PIf);
    }

    return mag_heading;
}



// produce a yaw error value. The returned value is proportional
// to sin() of the current heading error in earth frame
float yaw_error_compass(void)
{
float mag[3];
float rb[2];
float earth_mag[2]={cosf(magneticDeclination), sinf(magneticDeclination)};

mag[0]=(float)magADC[0];
mag[1]=(float)magADC[1];
mag[2]=(float)magADC[2];


//debugIMU=mag[0];
//debugIMU_1=mag[1];



float *temp_rb;

// get the mag vector in the earth frame
temp_rb=dcmMulXY(mag);

rb[0]=temp_rb[0];
rb[1]=temp_rb[1];




rb[0]/= sqrtf(sq(rb[0])+sq(rb[1]));
rb[1]/= sqrtf(sq(rb[0])+sq(rb[1]));


//debugIMU_2=rb[0]*100;
//debugIMU_3=rb[1]*100;

if(isinf(rb[0])||isinf(rb[1]))
{
    // not a valid vector
    return 0.0;


}



return ((rb[0]*earth_mag[1])-(rb[1]*earth_mag[0]));



}


// _yaw_gain reduces the gain of the PI controller applied to heading errors
// when observability from change of velocity is good (eg changing speed or turning)
// This reduces unwanted roll and pitch coupling due to compass errors for planes.
// High levels of noise on _accel_ef will cause the gain to drop and could lead to
// increased heading drift during straight and level flight, however some gain is
// always available. TODO check the necessity of adding adjustable acc threshold
// and/or filtering accelerations before getting magnitude
float yaw_gain(void)
{
    float VdotEFmag =  sqrtf(sq(accel_ef[0])+sq(accel_ef[1]));

    if (VdotEFmag <= 4.0f) {
        return 0.2f*(4.5f - VdotEFmag);
    }
    return 0.1f;
}





// yaw drift correction using the compass or GPS
// this function prodoces the _omega_yaw_P vector, and also
// contributes to the _omega_I.z long term yaw drift estimate
void drift_correction_yaw(void)
{
    bool new_value = false;
    float yaw_error;
    float yaw_deltat;


    if (compassLastUpdatedAt!= compass_last_update) {



    yaw_deltat = (compassLastUpdatedAt - compass_last_update) * 1.0e-6f;

    compass_last_update = compassLastUpdatedAt;


   if(!have_initial_yaw){
//   float heading = calculate_heading();
       float yaw_angle = calculate_heading();



       // debugIMU_4= degrees(heading);

   dcmFromEuler(anglerad[AI_ROLL], anglerad[AI_PITCH], yaw_angle);

   omega_yaw_P[0]=0;
   omega_yaw_P[1]=0;
   omega_yaw_P[2]=0;

   have_initial_yaw=true;
   }
   new_value = true;
   yaw_error = yaw_error_compass();




   }

    if (!new_value) {
            // we don't have any new yaw information
            // slowly decay _omega_yaw_P to cope with loss
            // of our yaw source
            omega_yaw_P[0] *= 0.97f;
            omega_yaw_P[1] *= 0.97f;
            omega_yaw_P[2] *= 0.97f;
            return;
        }

        // convert the error vector to body frame
        float error_z = dcm_matrix[2][2] * yaw_error;

        // the spin rate changes the P gain, and disables the
        // integration at higher rates
        float spin_rate = sqrtf(sq(omega[0])+sq(omega[1])+sq(omega[2]));;

        // sanity check _kp_yaw
        if (kp_yaw < YAW_P_MIN) {
            kp_yaw = YAW_P_MIN;
        }

        // update the proportional control to drag the
        // yaw back to the right value. We use a gain
        // that depends on the spin rate. See the fastRotations.pdf
        // paper from Bill Premerlani
        // We also adjust the gain depending on the rate of change of horizontal velocity which
        // is proportional to how observable the heading is from the acceerations and GPS velocity
        // The accelration derived heading will be more reliable in turns than compass or GPS

        omega_yaw_P[2] = error_z * pGain(spin_rate) * kp_yaw * yaw_gain();

        // disarm condition to handle
        if (!IS_RC_MODE_ACTIVE(BOXARM)) {
            omega_yaw_P[2] *= 8;
        }


        // don't update the drift term if we lost the yaw reference
        // for more than 2 seconds
        if (yaw_deltat < 2.0f && spin_rate < radians(SPIN_RATE_LIMIT)) {
            // also add to the I term
            omega_I_sum[2] += error_z * ki_yaw * yaw_deltat;
        }

        error_yaw_sum += fabsf(yaw_error);
        error_yaw_count++;



}




// perform drift correction. This function aims to update _omega_P and
// _omega_I with our best estimate of the short term and long term
// gyro error. The _omega_P value is what pulls our attitude solution
// back towards the reference vector quickly. The _omega_I term is an
// attempt to learn the long term drift rate of the gyros.
//
// This drift correction implementation is based on a paper
// by Bill Premerlani from here:
//   http://gentlenav.googlecode.com/files/RollPitchDriftCompensation.pdf
void drift_correction(float deltaT)
{

    // TODO move to imuCalculateAcceleration()
    float accG[3];

    //accG[0]=(float)accADC[0]/acc_1G;
    //accG[1]=(float)accADC[1]/acc_1G;
    //accG[2]=(float)accADC[2]/acc_1G;


    // perform yaw drift correction if we have a new yaw reference
    // vector
    drift_correction_yaw();



    // changing sign of accZ;
//    accADC[0]= -accADC[0];
//    accADC[1]= -accADC[1];
//    accADC[2]= -accADC[2];

    // rotate accelerometer values into the earth frame
    accel_ef[0]=((float)dcm_matrix[0][0]*(-accADC[0])) + ((float)dcm_matrix[0][1]*(-accADC[1])) + ((float)dcm_matrix[0][2]*(-accADC[2]));
    accel_ef[1]=((float)dcm_matrix[1][0]*(-accADC[0])) + ((float)dcm_matrix[1][1]*(-accADC[1])) + ((float)dcm_matrix[1][2]*(-accADC[2]));
    accel_ef[2]=((float)dcm_matrix[2][0]*(-accADC[0])) + ((float)dcm_matrix[2][1]*(-accADC[1])) + ((float)dcm_matrix[2][2]*(-accADC[2]));


//    debugIMU=accADC[0]*100;
//    debugIMU_1=accADC[1]*100;
//    debugIMU_2=accADC[2]*100;
//    debugIMU_3=accel_ef[0]*100;
//    debugIMU_4=accel_ef[1]*100;
//    debugIMU_5=accel_ef[2]*100;

    ra_sum[0]= accel_ef[0];
    ra_sum[1]= accel_ef[1];
    ra_sum[2]= accel_ef[2];


    ra_deltaT+=deltaT;


    // equation 9: get the corrected acceleration vector in earth frame. Units
    // are m/s/s
    float GA_e[3]= {0, 0, -1.0f};


    float ra_scale = 1.0f/(GRAVITY_CSS);

    // calculate the error term in earth frame.
    // we do this for each available accelerometer then pick the
    // accelerometer that leads to the smallest error term. This takes
    // advantage of the different sample rates on different
    // accelerometers to dramatically reduce the impact of aliasing
    // due to harmonics of vibrations that match closely the sampling
    // rate of our accelerometers. On the Pixhawk we have the LSM303D
    // running at 800Hz and the MPU6000 running at 1kHz, by combining
    // the two the effects of aliasing are greatly reduced.
    float error[3];
    float GA_b[3];


    ra_sum[0]*= ra_scale;
    ra_sum[1]*= ra_scale;
    ra_sum[2]*= ra_scale;

    GA_b[0] = ra_sum[0];
    GA_b[1] = ra_sum[1];
    GA_b[2] = ra_sum[2];

    // normalize
    GA_b[0]/= sqrtf(sq(GA_b[0])+sq(GA_b[1])+sq(GA_b[2]));
    GA_b[1]/= sqrtf(sq(GA_b[0])+sq(GA_b[1])+sq(GA_b[2]));
    GA_b[2]/= sqrtf(sq(GA_b[0])+sq(GA_b[1])+sq(GA_b[2]));




//
//    debugIMU=GA_b[0]*100;
//    debugIMU_1=GA_b[1]*100;
//    debugIMU_2=GA_b[2]*100;

  //  debugIMU_1=2;
    // cross product
    error[0]= GA_b[1]*GA_e[2] - GA_b[2]*GA_e[1];
    error[1]= GA_b[2]*GA_e[0] - GA_b[0]*GA_e[2];
    error[2]= GA_b[0]*GA_e[1] - GA_b[1]*GA_e[0];


//    debugIMU_3=error[0]*100;
//    debugIMU_4=error[1]*100;
//    debugIMU_5=error[2]*100;


    // convert the error term to body frame

    float *temp_error;
    temp_error=dcmMulTranspose(error);

    error[0]=temp_error[0];
    error[1]=temp_error[1];
    error[2]=temp_error[2];


    // base the P gain on the spin rate
    float spin_rate = sqrtf(sq(omega[0])+sq(omega[1])+sq(omega[2]));

    // sanity check _kp value
    if (kp < RP_P_MIN) {
        kp = RP_P_MIN;
    }

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    omega_P[0] = error[0] * pGain(spin_rate) * kp;
    omega_P[1] = error[1] * pGain(spin_rate) * kp;
    omega_P[2] = error[2] * pGain(spin_rate) * kp;

   // TODO handling unarmed condition
   if (!IS_RC_MODE_ACTIVE(BOXARM)){
        omega_P[0] *= 8;
        omega_P[1] *= 8;
        omega_P[2] *= 8;
    }



    // accumulate some integrator error
    if (spin_rate < radians(SPIN_RATE_LIMIT)) {
       omega_I_sum[0] += error[0] * ki * ra_deltaT;
       omega_I_sum[1] += error[1] * ki * ra_deltaT;
       omega_I_sum[2] += error[2] * ki * ra_deltaT;

       omega_I_sum_time += ra_deltaT;
    }

    if (omega_I_sum_time >= 5) {
       // limit the rate of change of omega_I to the hardware
       // reported maximum gyro drift rate. This ensures that
       // short term errors don't cause a buildup of omega_I
       // beyond the physical limits of the device


       float change_limit = gyro_drift_limit * omega_I_sum_time;



       omega_I_sum[0] = constrainf(omega_I_sum[0], -change_limit, change_limit);
       omega_I_sum[1] = constrainf(omega_I_sum[1], -change_limit, change_limit);
       omega_I_sum[2] = constrainf(omega_I_sum[2], -change_limit, change_limit);

       omega_I[0] += omega_I_sum[0];
       omega_I[1] += omega_I_sum[1];
       omega_I[2] += omega_I_sum[2];

       omega_I_sum[0]=0;
       omega_I_sum[1]=0;
       omega_I_sum[2]=0;

       omega_I_sum_time = 0;
    }

 //   debugIMU_2=3;

//    debugIMU=omega_P[0]*1000;
//    debugIMU_1=omega_P[1]*1000;
//    debugIMU_2=omega_P[2]*1000;
//    debugIMU_3=omega_I[0]*1000000;
//    debugIMU_4=omega_I[1]*1000000;
//    debugIMU_5=omega_I[2]*1000000;
//    debugIMU_6=omega_I[0]*100000;
//    debugIMU_7=omega_I[1]*100000;
//    debugIMU_8=omega_I[2]*100000;




    ra_sum[0]=0;
    ra_sum[1]=0;
    ra_sum[2]=0;

    ra_deltaT=0;

}

#endif




void imuUpdate(rollAndPitchTrims_t *accelerometerTrims)
{
    #ifdef DCM
      static uint32_t previousT;
      uint32_t currentT;
      float deltaT;
    #endif

    gyroUpdate();


    #ifdef DCM
    // calculate deltaT after gyro update
      currentT = micros();
      deltaT =(float)(( currentT - previousT)/1000000.0f); // micro-sec to sec
      previousT = currentT;


      if (deltaT > 0.2f) {
             // memset(&_ra_sum[0], 0, sizeof(_ra_sum));
             // _ra_deltat = 0;

              ra_sum[0]=0;
              ra_sum[1]=0;
              ra_sum[2]=0;

              ra_deltaT=0;

              return;
          }
    #endif


    if (sensors(SENSOR_ACC)) {
        updateAccelerationReadings(accelerometerTrims); // TODO rename to accelerometerUpdate and rename many other 'Acceleration' references to be 'Accelerometer'


        #ifdef DCM

        // Integrate the DCM matrix using gyro inputs
             matrix_update(deltaT);

             // Normalize the DCM matrix
             normalize();

             // Perform drift correction
             drift_correction(deltaT);

             // paranoid check for bad values in the DCM matrix
             check_matrix();

             // Calculate pitch, roll, yaw for stabilization and navigation
             euler_angles();

             inclination.values.rollDeciDegrees = lrintf(anglerad[AI_ROLL] * (1800.0f / M_PIf));
             inclination.values.pitchDeciDegrees = lrintf(anglerad[AI_PITCH] * (1800.0f / M_PIf));

             inclination_generalised.values.pitchDeciDegrees=  inclination.values.pitchDeciDegrees;
             inclination_generalised.values.rollDeciDegrees=  inclination.values.rollDeciDegrees;



             float hd = (-anglerad[AI_YAW]  * 1800.0f / M_PIf + magneticDeclination) / 10.0f;


             heading = lrintf(hd);


                // Arctan returns a value in the range -180 to 180 degrees. We 'normalize' negative angles to be positive.
                if (heading < 0)
                    heading += 360;



                uint8_t axis = 0;
                int32_t accMag = 0;
                static float accLPF[3];

                   for (axis = 0; axis < 3; axis++) {
                           //deltaGyroAngle.raw[axis] = gyroADC[axis] * scale;
                           if (imuRuntimeConfig->acc_lpf_factor > 0) {
                               accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / imuRuntimeConfig->acc_lpf_factor)) + accADC[axis] * (1.0f / imuRuntimeConfig->acc_lpf_factor);
                               accSmooth[axis] = (int16_t) accLPF[axis];
                           } else {
                               accSmooth[axis] = accADC[axis];
                           }

                           //netAccMagnitude=accSmooth[axis]  ;

                           accMag += ((int32_t) accSmooth[axis] * (int32_t) accSmooth[axis]) / (int32_t) acc_1G;
                       }

                       accMag = accMag * 100 / (int32_t) acc_1G;

                       netAccMagnitude = accMag;

                   imuCalculateAcceleration(deltaT*1000000);//TODO fix this and optimise



            if (ABS(inclination_generalised.values.pitchDeciDegrees) < smallAngle &&ABS(inclination_generalised.values.rollDeciDegrees) < smallAngle ) {


                   ENABLE_STATE(SMALL_ANGLE);

              } else {
                  DISABLE_STATE(SMALL_ANGLE);
              }
        #else
            imuCalculateEstimatedAttitude();

        #endif
    } else {
        accADC[X] = 0;
        accADC[Y] = 0;
        accADC[Z] = 0;
    }
}

int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value)
{
    float cosZ = EstG.V.Z / sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z);

    /*
     * Use 0 as the throttle angle correction if we are inverted, vertical or with a
     * small angle < 0.86 deg
     * TODO: Define this small angle in config.
     */
    if (cosZ <= 0.015f) {
        return 0;
    }
    int angle = lrintf(acosf(cosZ) * throttleAngleScale);
    if (angle > 900)
        angle = 900;
    return lrintf(throttle_correction_value * sin_approx(angle / (900.0f * M_PIf / 2.0f)));
}
