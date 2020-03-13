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
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/ranging_vl53l0x.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/filter.h"
#include "flight/acrobats.h"
#include "flight/posEstimate.h"
#include "flight/posControl.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "opticflow.h"

#include "../drivers/opticflow_paw3903.h"
#include "mw.h"

#define UPDATE_FREQUENCY (1000 * 10) //100Hz
#define SENSORFLOW_LPS 0.05

// minimum assumed height
const float height_min = 0.1;

// maximum scaling height
const float height_max = 3.0;

float delta_velocity[3];

float delta_velocity_ne[2];
// last flow rate in radians/sec in north-east axis
float last_flow_rate_rps[2];

float flow_rate_rps[2] = { 0 };

// timestamp of last flow data
uint32_t last_flow_ms;

float last_ins_height;
float height_offset;

float filtered_raw_flow[2] = { 0 };
float sensor_flow[3] = { 0 };
float sensor_flow_hf[2] = { 0 };

float accel_hf[3] = { 0 };
float accel_hf_prev[3] = { 0 };
float velocity_hf[3] = { 0 };
float opticflowHeight = 0;

float flow_max = 0.6;

float debugOpticFlowVar=0;
float debugOpticFlowVar1=0;
float debugOpticFlowVar2=0;
float debugOpticFlowVar3=0;
float debugOpticFlowVar4=0;
float debugOpticFlow[2];
float debugOpticFlow1[2];
float debugOpticFlow2[3];





void updateHeightEstimate(uint32_t currentTime)
{

    float *tempVector;

    static uint32_t previousTime;

    float ins_height = altitudeHoldGetEstimatedAltitude() * 0.01;

    if (!ARMING_FLAG(ARMED) || (int32_t)(millis() - arm_time) < 2500) {

        height_offset = -ins_height;
        last_ins_height = ins_height;

        return;

    }

    float dt = (currentTime - previousTime) / 1000000.0f;
    uint32_t dTime;
    dTime = currentTime - previousTime;

    if (dTime < UPDATE_FREQUENCY)
        return;

    previousTime = currentTime;

    /* Sanity Check */
    if (dTime > 2 * UPDATE_FREQUENCY) {       //Too long. Reset things.
        imuResetAccelerationSum(0);
    }

    if (accSumCountXYZ) {

        accel_hf_prev[0] = accel_hf[0];
        accel_hf_prev[1] = accel_hf[1];
        accel_hf[0] = (float) accSumXYZ[0] / (float) accSumCountXYZ;
        accel_hf[1] = (float) accSumXYZ[1] / (float) accSumCountXYZ;

    } else {
        accel_hf[0] = 0;
        accel_hf[1] = 0;

    }

    imuResetAccelerationSum(0);

    delta_velocity_ne[0] += (accel_hf[0] * (9.80665f / acc_1G)) * dt;
    delta_velocity_ne[1] += (accel_hf[1] * (9.80665f / acc_1G)) * dt;

    accel_hf[0] = accel_hf[0] * accVelScale;
    accel_hf[1] = (accel_hf[1] * accVelScale);

    if (last_flow_ms == 0) {
        // just starting up
        last_flow_ms = last_opticflow_update_ms;
        delta_velocity_ne[0] = 0;
        delta_velocity_ne[1] = 0;
        height_offset = 0;

        return;
    }

    if (last_opticflow_update_ms == last_flow_ms) {
        // no new flow data
        return;
    }

    float delta_vel_bf[2];
    float delta_vel_rate[2];

    delta_vel_bf[0] = delta_velocity_ne[0];
    delta_vel_bf[1] = delta_velocity_ne[1];

    delta_vel_rate[0] = delta_vel_bf[0];
    delta_vel_rate[1] = delta_vel_bf[1];

    flow_rate_rps[0] = flowRate[0] + bodyRate[1];
    flow_rate_rps[0] = -(flowRate[1] + bodyRate[0]);

    uint32_t dt_ms = last_opticflow_update_ms - last_flow_ms;

    if (dt_ms > 500) {
        // too long between updates, ignore
        last_flow_ms = last_opticflow_update_ms;
        delta_velocity_ne[0] = 0;
        delta_velocity_ne[1] = 0;

        last_flow_rate_rps[0] = flow_rate_rps[0];
        last_flow_rate_rps[1] = flow_rate_rps[1];

        last_ins_height = ins_height;
        height_offset = 0;
        return;
    }

    float delta_flowrate[2];

    delta_flowrate[0] = flow_rate_rps[0] - last_flow_rate_rps[0];
    delta_flowrate[1] = flow_rate_rps[1] - last_flow_rate_rps[1];

    last_flow_rate_rps[0] = flow_rate_rps[0];
    last_flow_rate_rps[1] = flow_rate_rps[1];

    last_flow_ms = last_opticflow_update_ms;

    tempVector = dcmBodyToEarth3D(delta_flowrate);

    delta_flowrate[0] = tempVector[0];
    delta_flowrate[1] = tempVector[1];

    //  update height estimate

    const float min_velocity_change = 0.04;
    const float min_flow_change = 0.04;
    const float height_delta_max = 0.25;

    //  for each axis update the height estimate

    float delta_height = 0;
    uint8_t total_weight = 0;
    float height_estimate = ins_height + height_offset;

    for (uint8_t i = 0; i < 2; i++) {
        // only use height estimates when we have significant delta-velocity and significant delta-flow
        float abs_flow = fabsf(delta_flowrate[i]);
        if (abs_flow < min_flow_change || fabsf(delta_vel_rate[i]) < min_velocity_change) {
            continue;
        }
        // get instantaneous height estimate
        float height = delta_vel_rate[i] / delta_flowrate[i];

        if (height <= 0) {
            // discard negative heights
            continue;
        }
        delta_height += (height - height_estimate) * abs_flow;
        total_weight += abs_flow;
    }
    if (total_weight > 0) {
        delta_height /= total_weight;
    }

    if (delta_height < 0) {
        // bias towards lower heights, as we'd rather have too low
        // gain than have oscillation. This also compensates a bit for
        // the discard of negative heights above
        delta_height *= 2;
    }

    // don't update height by more than height_delta_max, this is a simple way of rejecting noise
    float new_offset = height_offset + constrainf(delta_height, -height_delta_max, height_delta_max);

    // apply a simple filter
    height_offset = 0.8 * height_offset + 0.2 * new_offset;

    if (ins_height + height_offset < height_min) {
        // height estimate is never allowed below the minimum
        height_offset = height_min - ins_height;
    }

    // new height estimate for logging
    height_estimate = ins_height + height_offset;

    opticflowHeight = height_estimate;

    last_ins_height = ins_height;

    delta_velocity_ne[0] = 0;
    delta_velocity_ne[1] = 0;

}

void calculateSensorFlow(uint32_t currentTime)
{

    float *tempVector;
    static uint32_t previousTime;

    float dt = (currentTime - previousTime) / 1000000.0f;

    uint32_t dTime;
    dTime = currentTime - previousTime;

    if (dTime < UPDATE_FREQUENCY)
        return;

    previousTime = currentTime;

    /* Sanity Check */
//       if (dTime > 2 * UPDATE_FREQUENCY) {       //Too long. Reset things.
//           imuResetAccelerationSum(0);
//       }


    float raw_flow[2];

    raw_flow[0] = flowRate[0] + bodyRate[1];
    raw_flow[1] = -(flowRate[1] + bodyRate[0]);

    debugOpticFlow[0]=raw_flow[0];
    debugOpticFlow[1]=raw_flow[1];


//    raw_flow[0]=constrainf(raw_flow[0], -flow_max, flow_max);
//    raw_flow[1]=constrainf(raw_flow[1], -flow_max, flow_max);

    filtered_raw_flow[0] = (filtered_raw_flow[0] * (1 - SENSORFLOW_LPS)) + (raw_flow[0] * SENSORFLOW_LPS);
    filtered_raw_flow[1] = (filtered_raw_flow[1] * (1 - SENSORFLOW_LPS)) + (raw_flow[1] * SENSORFLOW_LPS);

//    sensor_flow[0]=filtered_raw_flow[0]*constrainf((opticflowHeight), height_min, height_max);
//    sensor_flow[1]=filtered_raw_flow[1]*constrainf((opticflowHeight), height_min, height_max);

    sensor_flow[0] = filtered_raw_flow[0] * constrainf((NewSensorRange * 0.001), height_min, height_max);
    sensor_flow[1] = filtered_raw_flow[1] * constrainf((NewSensorRange * 0.001), height_min, height_max);


    debugOpticFlow1[0]=filtered_raw_flow[0];
    debugOpticFlow1[1]=filtered_raw_flow[1];



    tempVector = dcmBodyToEarth3D(sensor_flow);              //hbf

    sensor_flow_hf[0] = tempVector[0];
    sensor_flow_hf[1] = tempVector[1];

    if (ARMING_FLAG(ARMED)) {

        VelocityX = sensor_flow_hf[0] * 100;
        VelocityY = sensor_flow_hf[1] * 100;

        PositionX += (VelocityX * dt) + (0.5 * accel_hf[0]) * (dt * dt);
        PositionY += (VelocityY * dt) + (0.5 * accel_hf[1]) * (dt * dt);
    } else {

        VelocityX = 0;
        VelocityY = 0;
        PositionX = 0;
        PositionY = 0;
    }

}

void runFlowHold(uint32_t currentTime)
{

    updateHeightEstimate(currentTime);
    calculateSensorFlow(currentTime);

}


