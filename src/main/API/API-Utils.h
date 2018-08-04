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

#pragma once

#include <stdint.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/ranging_vl53l0x.h"
#include "drivers/gpio.h"

#include "API/Hardware/Specifiers.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RESET_CHECK 3

extern uint8_t resetCounter;
extern uint8_t intLogCounter;
extern uint8_t floatLogCounter;
extern int8_t userRCflag[4];
extern int16_t appHeading;
extern int16_t AUX3_VALUE;
extern int16_t userHeading;
extern uint16_t userLoopFrequency;
extern int32_t user_GPS_coord[2];
extern uint32_t autoRcTimerLoop;
extern int32_t MOTOR_ARRAY[4];
extern int32_t app_GPS_coord[2];
extern int32_t RC_ARRAY[4];


extern bool runUserCode;
extern bool useAutoRC;
extern bool External_RC_FLAG[4];
extern bool callibrateAccelero;
extern bool FlightStatusEnabled;
extern bool hasTakeOff;
extern bool AutoAccCalibration;
extern bool callOnPilotStart;
extern bool callonPilotFinish;
extern bool fsLowBattery;
extern bool fsInFlightLowBattery;
extern bool fsCrash;
extern bool isUserHeadingSet;
extern bool isUserGPSCoordSet;
extern bool startShieldRanging;
extern bool reverseMode;
extern bool reverseReferenceFrame;
extern bool motorMixer;
extern bool isLocalisationOn;
extern bool DONT_USE_STATUS_LED;

extern LaserSensor* laser_sensors[3];


void setM1GPIO(bool direction);
void setM2GPIO(bool direction);
void setM3GPIO(bool direction);
void setM4GPIO(bool direction);

void userEnabledLand();
void resetUserRCflag(void);


#define UB_ADC1_CHANNEL_COUNT 2
#define UB_ADC2_CHANNEL_COUNT 2
#define UB_ADC3_CHANNEL_COUNT 1
#define UB_ADC4_CHANNEL_COUNT 4

typedef enum {
    UB_ADC_IN1 = 0,
    UB_ADC_IN2 = 1,
    UB_ADC_IN3 = 2,

} unibus_AdcChannel;

#define ADC_CHANNEL_COUNT (ADC_CHANNEL_MAX + 1)

typedef struct unibus_adc_config_s {
    uint8_t dmaIndex;        // index into DMA buffer in case of sparse channels
    bool enabled;
} unibus_adc_config_t;

extern unibus_adc_config_t adc1Config[UB_ADC1_CHANNEL_COUNT];
extern unibus_adc_config_t adc2Config[UB_ADC2_CHANNEL_COUNT];
extern unibus_adc_config_t adc3Config[UB_ADC3_CHANNEL_COUNT];
extern unibus_adc_config_t adc4Config[UB_ADC4_CHANNEL_COUNT];

extern volatile uint16_t adc1Values[UB_ADC1_CHANNEL_COUNT];
extern volatile uint16_t adc2Values[UB_ADC2_CHANNEL_COUNT];
extern volatile uint16_t adc3Values[UB_ADC3_CHANNEL_COUNT];
extern volatile uint16_t adc4Values[UB_ADC4_CHANNEL_COUNT];


void unibusAdcConfig();
void unibusAdcInit();

int getGPIOport(unibus_e pin);
GPIO_Pin getGPIOpin(unibus_e pin);
uint32_t getGPIOclock(unibus_e pin);
uint8_t getGPIOpinSource(unibus_e pin);

uint16_t getTimerCh(unibus_e pin);
uint8_t getADCCh(unibus_e pin);

void reverseMotorGPIOInit();

void resetUser();

#ifdef __cplusplus
}
#endif
