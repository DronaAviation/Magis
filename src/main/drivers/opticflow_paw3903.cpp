/*
 * This file is part of Magis.
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
#include "flight/opticflow.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "API/Utils.h"
#include "API/Peripheral.h"

#include "opticflow_paw3903.h"

#define DISABLE_SPI       GPIO_SetBits(GPIOB,   GPIO_Pin_5)
#define ENABLE_SPI        GPIO_ResetBits(GPIOB, GPIO_Pin_5)

#define CXOF_HEADER         (uint8_t)0xFE
#define CXOF_FOOTER         (uint8_t)0xAA
#define CXOF_FRAME_LENGTH               9
#define CXOF_PIXEL_SCALING1      (1.76e-3)
#define CXOF_PIXEL_SCALING        0.00745
#define CXOF_TIMEOUT_SEC             0.3f

int16_t flowScalerX = 0;
int16_t flowScalerY = 0;

uint32_t last_frame_us;             // system time of last message from flow sensor
uint8_t buf[12];                    // buff of characters received from flow sensor
uint8_t buf_len;                    // number of characters in buffer
float gyro_sum[2];                  // sum of gyro sensor values since last frame from flow sensor
uint16_t gyro_sum_count;            // number of gyro sensor values in sum
uint32_t last_opticflow_update_ms;

uint8_t surface_quality;
float flowRate[2];
float bodyRate[2];
float bodyRate1[2];

//Interval updateTimer;

float hist_gyroX;
float hist_gyroY;

uint8_t motion = 0;

Interval opticInterval;

#define MAXIMUM_QUEUE_SIZE 3

static float bufferX[MAXIMUM_QUEUE_SIZE], bufferY[MAXIMUM_QUEUE_SIZE];
static int16_t head = 0;
static int16_t rear = -1;
static int16_t itemCount = 0;

uint8_t opticFlowAddress = 0;

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

void mode_0_init()
{

    spi.Write(0x7F, 0x00);
    spi.Write(0x55, 0x01);
    spi.Write(0x50, 0x07);
    spi.Write(0x7F, 0x0E);
    spi.Write(0x43, 0x10);
    spi.Write(0x48, 0x02);
    spi.Write(0x7F, 0x00);
    spi.Write(0x51, 0x7B);
    spi.Write(0x50, 0x00);
    spi.Write(0x55, 0x00);
    spi.Write(0x7F, 0x00);

    spi.Write(0x61, 0xAD);
    spi.Write(0x7F, 0x03);
    spi.Write(0x40, 0x00);
    spi.Write(0x7F, 0x05);
    spi.Write(0x41, 0xB3);
    spi.Write(0x43, 0xF1);
    spi.Write(0x45, 0x14);
    spi.Write(0x5F, 0x34);
    spi.Write(0x7B, 0x08);
    spi.Write(0x5E, 0x34);
    spi.Write(0x5B, 0x32);
    spi.Write(0x6D, 0x32);
    spi.Write(0x45, 0x17);
    spi.Write(0x70, 0xE5);
    spi.Write(0x71, 0xE5);
    spi.Write(0x7F, 0x06);
    spi.Write(0x44, 0x1B);
    spi.Write(0x40, 0xBF);
    spi.Write(0x4E, 0x3F);
    spi.Write(0x7F, 0x08);
    spi.Write(0x66, 0x44);
    spi.Write(0x65, 0x20);
    spi.Write(0x6A, 0x3A);
    spi.Write(0x61, 0x05);
    spi.Write(0x62, 0x05);
    spi.Write(0x7F, 0x09);
    spi.Write(0x4F, 0xAF);
    spi.Write(0x48, 0x80);
    spi.Write(0x49, 0x80);
    spi.Write(0x57, 0x77);
    spi.Write(0x5F, 0x40);
    spi.Write(0x60, 0x78);
    spi.Write(0x61, 0x78);
    spi.Write(0x62, 0x08);
    spi.Write(0x63, 0x50);
    spi.Write(0x7F, 0x0A);
    spi.Write(0x45, 0x60);
    spi.Write(0x7F, 0x00);

    spi.Write(0x4D, 0x11);
    spi.Write(0x55, 0x80);
    spi.Write(0x74, 0x21);
    spi.Write(0x75, 0x1F);
    spi.Write(0x4A, 0x78);
    spi.Write(0x4B, 0x78);
    spi.Write(0x44, 0x08);
    spi.Write(0x45, 0x50);
    spi.Write(0x64, 0xFE);
    spi.Write(0x65, 0x1F);
    spi.Write(0x72, 0x0A);
    spi.Write(0x73, 0x00);
    spi.Write(0x7F, 014);
    spi.Write(0x44, 0x84);
    spi.Write(0x65, 0x47);
    spi.Write(0x66, 0x18);
    spi.Write(0x63, 0x70);
    spi.Write(0x6F, 0x2C);
    spi.Write(0x7F, 0x15);
    spi.Write(0x48, 0x48);
    spi.Write(0x7F, 0x07);
    spi.Write(0x41, 0x0D);
    spi.Write(0x43, 0x14);
    spi.Write(0x4B, 0x0E);
    spi.Write(0x45, 0x0F);
    spi.Write(0x44, 0x42);
    spi.Write(0x4C, 0x80);
    spi.Write(0x7F, 0x10);
    spi.Write(0x5B, 0x03);
    spi.Write(0x7F, 0x07);
    spi.Write(0x40, 0x41);

    delay(10);

    spi.Write(0x7F, 0x00);
    spi.Write(0x32, 0x00);
    spi.Write(0x7F, 0x07);
    spi.Write(0x40, 0x40);
    spi.Write(0x7F, 0x06);
    spi.Write(0x68, 0x70);

    spi.Write(0x69, 0x01);
    spi.Write(0x7F, 0x0D);
    spi.Write(0x48, 0xC0);
    spi.Write(0x6F, 0xD5);
    spi.Write(0x7F, 0x00);
    spi.Write(0x5B, 0xA0);
    spi.Write(0x4E, 0xA8);
    spi.Write(0x5A, 0x50);
    spi.Write(0x40, 0x80);
    spi.Write(0x73, 0x1F);

    delay(10);

    spi.Write(0x73, 0x00);

}

void mode_1_init()
{

    spi.Write(0x7F, 0x00);
    spi.Write(0x55, 0x01);
    spi.Write(0x50, 0x07);
    spi.Write(0x7F, 0x0E);
    spi.Write(0x43, 0x10);
    spi.Write(0x48, 0x02);
    spi.Write(0x7F, 0x00);
    spi.Write(0x51, 0x7B);
    spi.Write(0x50, 0x00);
    spi.Write(0x55, 0x00);
    spi.Write(0x7F, 0x00);
    spi.Write(0x61, 0xAD);
    spi.Write(0x7F, 0x03);
    spi.Write(0x40, 0x00);
    spi.Write(0x7F, 0x05);
    spi.Write(0x41, 0xB3);
    spi.Write(0x43, 0xF1);
    spi.Write(0x45, 0x14);
    spi.Write(0x5F, 0x34);
    spi.Write(0x7B, 0x08);

    spi.Write(0x5E, 0x34);
    spi.Write(0x5B, 0x65);
    spi.Write(0x6D, 0x65);
    spi.Write(0x45, 0x17);
    spi.Write(0x70, 0xE5);
    spi.Write(0x71, 0xE5);
    spi.Write(0x7F, 0x06);
    spi.Write(0x44, 0x1B);
    spi.Write(0x40, 0xBF);
    spi.Write(0x4E, 0x3F);
    spi.Write(0x7F, 0x08);
    spi.Write(0x66, 0x44);
    spi.Write(0x65, 0x20);
    spi.Write(0x6A, 0x3A);
    spi.Write(0x61, 0x05);
    spi.Write(0x62, 0x05);
    spi.Write(0x7F, 0x09);
    spi.Write(0x4F, 0xAF);
    spi.Write(0x48, 0x80);
    spi.Write(0x49, 0x80);
    spi.Write(0x57, 0x77);
    spi.Write(0x5F, 0x40);
    spi.Write(0x60, 0x78);
    spi.Write(0x61, 0x78);
    spi.Write(0x62, 0x08);
    spi.Write(0x63, 0x50);
    spi.Write(0x7F, 0x0A);
    spi.Write(0x45, 0x60);
    spi.Write(0x7F, 0x00);
    spi.Write(0x4D, 0x11);
    spi.Write(0x55, 0x80);
    spi.Write(0x74, 0x21);
    spi.Write(0x75, 0x1F);
    spi.Write(0x4A, 0x78);
    spi.Write(0x4B, 0x78);
    spi.Write(0x44, 0x08);
    spi.Write(0x45, 0x50);
    spi.Write(0x64, 0xFE);

    spi.Write(0x65, 0x1F);
    spi.Write(0X72, 0x0A);
    spi.Write(0x73, 0x00);
    spi.Write(0x7F, 0x14);
    spi.Write(0x44, 0x84);
    spi.Write(0x65, 0x67);
    spi.Write(0x66, 0x18);
    spi.Write(0x63, 0x70);
    spi.Write(0x6F, 0x2C);
    spi.Write(0x7F, 0x15);
    spi.Write(0x48, 0x48);
    spi.Write(0x75, 0x07);
    spi.Write(0x41, 0x0D);
    spi.Write(0x43, 0x14);
    spi.Write(0x4B, 0x0E);
    spi.Write(0x45, 0x0F);
    spi.Write(0x44, 0x42);
    spi.Write(0x4C, 0x80);
    spi.Write(0x7F, 0x10);
    spi.Write(0x5B, 0x03);
    spi.Write(0x74, 0x07);
    spi.Write(0x40, 0x41);

    delay(10);

    spi.Write(0x7F, 0x00);
    spi.Write(0x32, 0x00);
    spi.Write(0x7F, 0x07);
    spi.Write(0x40, 0x40);
    spi.Write(0x7F, 0x06);
    spi.Write(0x68, 0x70);
    spi.Write(0x69, 0x01);
    spi.Write(0x7F, 0x0D);
    spi.Write(0x48, 0xC0);
    spi.Write(0x6F, 0xD5);
    spi.Write(0x7F, 0x00);
    spi.Write(0x5B, 0xA0);
    spi.Write(0x4E, 0xA8);
    spi.Write(0x5A, 0x50);
    spi.Write(0x40, 0x80);

    spi.Write(0x73, 0x1F);

    delay(10);

    spi.Write(0x73, 0x00);

}

static void initRegisters()
{
    spi.Write(0x7F, 0x00);
    spi.Write(0x61, 0xAD);
    spi.Write(0x7F, 0x03);
    spi.Write(0x40, 0x00);
    spi.Write(0x7F, 0x05);
    spi.Write(0x41, 0xB3);
    spi.Write(0x43, 0xF1);
    spi.Write(0x45, 0x14);
    spi.Write(0x5B, 0x32);
    spi.Write(0x5F, 0x34);
    spi.Write(0x7B, 0x08);
    spi.Write(0x7F, 0x06);
    spi.Write(0x44, 0x1B);
    spi.Write(0x40, 0xBF);
    spi.Write(0x4E, 0x3F);
    spi.Write(0x7F, 0x08);
    spi.Write(0x65, 0x20);
    spi.Write(0x6A, 0x18);
    spi.Write(0x7F, 0x09);
    spi.Write(0x4F, 0xAF);
    spi.Write(0x5F, 0x40);
    spi.Write(0x48, 0x80);
    spi.Write(0x49, 0x80);
    spi.Write(0x57, 0x77);
    spi.Write(0x60, 0x78);
    spi.Write(0x61, 0x78);
    spi.Write(0x62, 0x08);
    spi.Write(0x63, 0x50);
    spi.Write(0x7F, 0x0A);
    spi.Write(0x45, 0x60);
    spi.Write(0x7F, 0x00);
    spi.Write(0x4D, 0x11);
    spi.Write(0x55, 0x80);
    spi.Write(0x74, 0x1F);
    spi.Write(0x75, 0x1F);
    spi.Write(0x4A, 0x78);
    spi.Write(0x4B, 0x78);
    spi.Write(0x44, 0x08);
    spi.Write(0x45, 0x50);
    spi.Write(0x64, 0xFF);
    spi.Write(0x65, 0x1F);
    spi.Write(0x7F, 0x14);
    spi.Write(0x65, 0x67);
    spi.Write(0x66, 0x08);
    spi.Write(0x63, 0x70);
    spi.Write(0x7F, 0x15);
    spi.Write(0x48, 0x48);
    spi.Write(0x7F, 0x07);
    spi.Write(0x41, 0x0D);
    spi.Write(0x43, 0x14);
    spi.Write(0x4B, 0x0E);
    spi.Write(0x45, 0x0F);
    spi.Write(0x44, 0x42);
    spi.Write(0x4C, 0x80);
    spi.Write(0x7F, 0x10);
    spi.Write(0x5B, 0x02);
    spi.Write(0x7F, 0x07);
    spi.Write(0x40, 0x41);
    spi.Write(0x70, 0x00);

    delay(10); // delay 10ms

    spi.Write(0x32, 0x44);
    spi.Write(0x7F, 0x07);
    spi.Write(0x40, 0x40);
    spi.Write(0x7F, 0x06);
    spi.Write(0x62, 0xF0);
    spi.Write(0x63, 0x00);
    spi.Write(0x7F, 0x0D);
    spi.Write(0x48, 0xC0);
    spi.Write(0x6F, 0xD5);
    spi.Write(0x7F, 0x00);
    spi.Write(0x5B, 0xA0);
    spi.Write(0x4E, 0xA8);
    spi.Write(0x5A, 0x50);
    spi.Write(0x40, 0x80);

    spi.Write(0x7F, 0x00);
    spi.Write(0x5A, 0x10);
    spi.Write(0x54, 0x00);
}

void initOpticFlow()
{

    GPIO.init(Pin14, OUTPUT);
    GPIO.write(Pin14, STATE_HIGH);

    delay(40);

    GPIO.write(Pin14, STATE_HIGH);
    delay(2);
    GPIO.write(Pin14, STATE_LOW);
    delay(2);
    GPIO.write(Pin14, STATE_HIGH);
    delay(2);

    opticFlowAddress = spi.Read(0x00, 1);

    spi.Write(0x3A, 0x5A);

    delay(5);

    spi.Read(0x02, 1);
    spi.Read(0x03, 1);
    spi.Read(0x04, 1);
    spi.Read(0x05, 1);
    spi.Read(0x06, 1);

    delay(1);

    mode_0_init();

    delay(100);


}

void updateOpticFlow()
{

//    if(!updateTimer.set(40, true))
//        return;
//

    // record gyro values as long as they are being used
    // the sanity check of dt below ensures old gyro values are not used
    if (gyro_sum_count < 1000) {

        gyro_sum[0] += (gyroADC[0] * 0.00106);
        gyro_sum[1] += (gyroADC[1] * 0.00106);
        gyro_sum_count++;
    }

    // sensor values
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    uint16_t qual_sum = 0;
    uint16_t count = 0;

    while (UART.rxBytesWaiting(UART2)) {

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
                if (buf[buf_len - 1] != CXOF_FOOTER) {
                    buf_len = 0;
                    continue;
                }

                // decode package
                int16_t x_raw = (int16_t)((uint16_t) buf[3] << 8) | buf[2];
                int16_t y_raw = (int16_t)((uint16_t) buf[5] << 8) | buf[4];

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
    last_opticflow_update_ms = millis();

    float dt = (this_frame_us - last_frame_us) * 1.0e-6;
    last_frame_us = this_frame_us;

    // sanity check dt
    if (is_positive(dt) && (dt < CXOF_TIMEOUT_SEC)) {
        // calculate flow values

        float flowScaleFactorX = 1.0f + 0.001f * flowScalerX;
        float flowScaleFactorY = 1.0f + 0.001f * flowScalerY;

        // copy flow rates to state structure
        flowRate[0] = ((float) x_sum / count) * flowScaleFactorX;

        flowRate[1] = (((float) y_sum / count) * flowScaleFactorY);

        flowRate[0] *= CXOF_PIXEL_SCALING / dt;

        flowRate[1] *= CXOF_PIXEL_SCALING / dt;


        if (isGyroXYQueueIsFull()) {
            getFrontGyroXY(&hist_gyroX, &hist_gyroY);
        } else {
            hist_gyroX = (gyro_sum[0] / gyro_sum_count);
            hist_gyroY = (gyro_sum[1] / gyro_sum_count);
        }

        bodyRate[0] = hist_gyroX;
        bodyRate[1] = hist_gyroY;

        addGyroXY(gyro_sum[0] / gyro_sum_count, gyro_sum[1] / gyro_sum_count);

    } else {
        // first frame received in some time so cannot calculate flow values
        flowRate[0] = 0;
        flowRate[1] = 0;

        bodyRate[0] = 0;
        bodyRate[1] = 0;

    }

    // reset gyro sum
    gyro_sum[0] = 0;
    gyro_sum[1] = 0;

    gyro_sum_count = 0;

}

void updateSpiOpticFlow()
{

    if (opticInterval.set(40, true)) {


        ENABLE_SPI;

        delayMicroseconds(50);

        spiTransferByte(SPI2, (0x16 & ~0x80u));

        delayMicroseconds(150);

        spiTransfer(SPI2, buf, NULL, 12);

        delayMicroseconds(50);


        DISABLE_SPI;

        delayMicroseconds(50);


        flowRate[0] = (int16_t)((uint16_t) buf[3] << 8) | buf[2];
        flowRate[1] = (int16_t)((uint16_t) buf[5] << 8) | buf[4];
        bodyRate1[0] = (int16_t) buf[6];
        bodyRate1[1] = (int16_t)((uint16_t) buf[10] << 8) | buf[11];


        uint32_t this_frame_us = micros();
        last_opticflow_update_ms = millis();

        float dt = (this_frame_us - last_frame_us);
        last_frame_us = this_frame_us;

        flowRate[0] *= (0.051);

        flowRate[1] *= (0.051);

        gyro_sum[0] = gyroADC[0] * 0.00106;
        gyro_sum[1] = gyroADC[1] * 0.00106;

        /*
        if (isGyroXYQueueIsFull()) {
            getFrontGyroXY(&hist_gyroX, &hist_gyroY);
        } else {
            hist_gyroX = gyro_sum[0];
            hist_gyroY = gyro_sum[1];
         }
         */

        bodyRate[0] = hist_gyroX;
        bodyRate[1] = hist_gyroY;

        bodyRate[0] = gyro_sum[0];
        bodyRate[1] = gyro_sum[1];

    }

}

