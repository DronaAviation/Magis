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


#include "opticflow_cheerson_cxof.h"
#include "../API/Utils.h"
#include "../API/Peripheral.h"





#define CXOF_HEADER         (uint8_t)0xFE
#define CXOF_FOOTER         (uint8_t)0xAA
#define CXOF_FRAME_LENGTH               9
#define CXOF_PIXEL_SCALING      (1.76e-3)
#define CXOF_TIMEOUT_SEC             0.3f



int16_t flowScalerX=0;
int16_t flowScalerY=0;


uint32_t last_frame_us;             // system time of last message from flow sensor
uint8_t buf[10];                    // buff of characters received from flow sensor
uint8_t buf_len;                    // number of characters in buffer
float gyro_sum[2];                  // sum of gyro sensor values since last frame from flow sensor
uint16_t gyro_sum_count;            // number of gyro sensor values in sum
uint32_t last_opticflow_update_ms;

uint8_t  surface_quality;
float flowRate[2];
float bodyRate[2];
float bodyRate1[2];

//Interval updateTimer;

float hist_gyroX;
float hist_gyroY;

#define MAXIMUM_QUEUE_SIZE 2

static float bufferX[MAXIMUM_QUEUE_SIZE], bufferY[MAXIMUM_QUEUE_SIZE];
static int16_t head = 0;
static int16_t rear = -1;
static int16_t itemCount = 0;



void addGyroXY(float gyroX, float gyroY)
{
    if (itemCount < MAXIMUM_QUEUE_SIZE) {
        rear++;
        if (rear >= MAXIMUM_QUEUE_SIZE) {
            rear = 0;
        }
        bufferX[rear] = gyroX;
        bufferY[rear] = gyroY;
        itemCount++;

    } else {
        if (++rear == MAXIMUM_QUEUE_SIZE) {
            rear = 0;
            bufferX[rear] = gyroX;
            bufferY[rear] = gyroY;
            head++;

        } else {

            bufferX[rear] = gyroX;
            bufferY[rear] = gyroY;
            head++;
            if (head == MAXIMUM_QUEUE_SIZE) {
                head = 0;
            }
        }
    }
}

void getFrontGyroXY(float *gyroX, float *gyroY)
{
    *gyroX = bufferX[head];
    *gyroY = bufferY[head];
     head++;

    if (head == MAXIMUM_QUEUE_SIZE) {
        head = 0;
    }
    itemCount--;
}

bool isGyroXYQueueIsFull(void)
{
    return itemCount == MAXIMUM_QUEUE_SIZE;
}




void initOpticFlow()
{

    UART.init(UART2,BAUD_RATE_19200);


}


void updateOpticFlow()
{



//    if(!updateTimer.set(40, true))
//        return;
//

    // record gyro values as long as they are being used
    // the sanity check of dt below ensures old gyro values are not used
    if (gyro_sum_count < 1000) {

        gyro_sum[0] += (gyroADC[0]*0.00106);
        gyro_sum[1] += (gyroADC[1]*0.00106);
        gyro_sum_count++;
    }


    // sensor values
   int32_t x_sum = 0;
   int32_t y_sum = 0;
   uint16_t qual_sum = 0;
   uint16_t count = 0;


    while(UART.rxBytesWaiting(UART2)){


        uint8_t c = UART.read8(UART2);


        if (buf_len == 0) {
                   if (c == CXOF_HEADER) {
                       buf[buf_len++] = c;
                   }
               } else {
                   // add character to buffer
                   buf[buf_len++] = c;

                   // if buffer has 9 items try to decode it
                   if (buf_len >= CXOF_FRAME_LENGTH) {
                       // check last character matches footer
                       if (buf[buf_len-1] != CXOF_FOOTER) {
                           buf_len = 0;
                           continue;
                       }

                       // decode package
                       int16_t x_raw = (int16_t)((uint16_t)buf[3] << 8) | buf[2];
                       int16_t y_raw = (int16_t)((uint16_t)buf[5] << 8) | buf[4];

                       // add to sum of all readings from sensor this iteration
                       count++;
                       x_sum += x_raw;
                       y_sum += y_raw;
                       qual_sum += buf[7];

                       // clear buffer
                       buf_len = 0;
                   }
               }

  }



    // return without updating state if no readings
       if (count == 0) {
           return;
       }

       // average surface quality scaled to be between 0 and 255
       surface_quality = (constrain(qual_sum / count, 64, 78) - 64) * 255 / 14;

           // calculate dt
           uint32_t this_frame_us = micros();
           last_opticflow_update_ms=millis();

           float dt = (this_frame_us - last_frame_us) * 1.0e-6;
           last_frame_us = this_frame_us;

           // sanity check dt
           if (is_positive(dt) && (dt < CXOF_TIMEOUT_SEC)) {
               // calculate flow values

               float flowScaleFactorX = 1.0f + 0.001f * flowScalerX;
               float flowScaleFactorY = 1.0f + 0.001f * flowScalerY;

               // copy flow rates to state structure
               flowRate[1] = ((float)x_sum / count) * flowScaleFactorX;

               flowRate[0] = (((float)y_sum / count) * flowScaleFactorY);

               flowRate[1] *= CXOF_PIXEL_SCALING / dt;

               flowRate[0] *= CXOF_PIXEL_SCALING / dt;


               //debug only
//               flowRate[0] *= CXOF_PIXEL_SCALING / dt;
//
//               flowRate[1] *= CXOF_PIXEL_SCALING / dt;

               // copy average body rate to state structure

//               float gyroLPF=App.getUserPID(Kp)/10;
//
               bodyRate1[0] = (gyro_sum[0] / gyro_sum_count);
               bodyRate1[1] = (gyro_sum[1] / gyro_sum_count);



               if(isGyroXYQueueIsFull()){

                   getFrontGyroXY(&hist_gyroX, &hist_gyroY);
               }else{

                   hist_gyroX=gyro_sum[0];
                   hist_gyroY=gyro_sum[1];
               }

               bodyRate[0] = hist_gyroX;
               bodyRate[1] = hist_gyroY;


               addGyroXY(gyro_sum[0], gyro_sum[1]);




//
//               _applyYaw(state.flowRate);
//               _applyYaw(state.bodyRate);
           } else {
               // first frame received in some time so cannot calculate flow values
               flowRate[0]=0;
               flowRate[1]=0;

               bodyRate[0]=0;
               bodyRate[1]=0;


               bodyRate1[0]=0;
               bodyRate1[1]=0;
           }



           // reset gyro sum
           gyro_sum[0]=0;
           gyro_sum[1]=0;

           gyro_sum_count = 0;



}
