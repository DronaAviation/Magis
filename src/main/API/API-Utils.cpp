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

#include "API-Utils.h"

#include <stdint.h>

#include "platform.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

#include "io/display.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"

#include "blackbox/blackbox.h"

#include "mw.h"


uint8_t resetCounter = 0;
uint8_t intLogCounter = 0;
uint8_t floatLogCounter = 0;
int8_t userRCflag[4] = { 0 };
int16_t appHeading = 0;
int16_t AUX3_VALUE = 1500;
int16_t userHeading = 0;
uint16_t userLoopFrequency = 100000;
uint32_t autoRcTimerLoop = 0;
int32_t user_GPS_coord[2];
int32_t MOTOR_ARRAY[4] = { 0 };
int32_t app_GPS_coord[2];
int32_t RC_ARRAY[4] = { 0 };



bool runUserCode = false;
bool useAutoRC = false;
bool External_RC_FLAG[4] = { true };
bool callibrateAccelero = true;
bool FlightStatusEnabled = true;
bool hasTakeOff = true;
bool AutoAccCalibration = false;
bool callOnPilotStart = true;
bool callonPilotFinish = false;
bool fsLowBattery = true;
bool fsInFlightLowBattery = true;
bool fsCrash = true;
bool isUserHeadingSet = false;
bool isUserGPSCoordSet = false;
bool startShieldRanging = false;
bool reverseMode = false;
bool reverseReferenceFrame = false;
bool motorMixer = true;
bool isLocalisationOn = false;
bool DONT_USE_STATUS_LED = false;


LaserSensor* laser_sensors[3] = { NULL };


unibus_adc_config_t adc1Config[UB_ADC1_CHANNEL_COUNT];
unibus_adc_config_t adc2Config[UB_ADC2_CHANNEL_COUNT];
unibus_adc_config_t adc3Config[UB_ADC3_CHANNEL_COUNT];
unibus_adc_config_t adc4Config[UB_ADC4_CHANNEL_COUNT];

volatile uint16_t adc1Values[UB_ADC1_CHANNEL_COUNT];
volatile uint16_t adc2Values[UB_ADC2_CHANNEL_COUNT];
volatile uint16_t adc3Values[UB_ADC3_CHANNEL_COUNT];
volatile uint16_t adc4Values[UB_ADC4_CHANNEL_COUNT];




void setM1GPIO(bool direction)
{

    if (direction)

        digitalHi(GPIOB, Pin_4);

    else
        digitalLo(GPIOB, Pin_4);

}

void setM2GPIO(bool direction)
{

    if (direction)

        digitalHi(GPIOA, Pin_7);

    else
        digitalLo(GPIOA, Pin_7);

}

void setM3GPIO(bool direction)
{

    if (direction)

        digitalHi(GPIOB, Pin_1);

    else
        digitalLo(GPIOB, Pin_1);

}

void setM4GPIO(bool direction)
{

    if (direction)

        digitalHi(GPIOA, Pin_15);

    else
        digitalLo(GPIOA, Pin_15);

}

void resetUserRCflag(void)
{
    if ((int32_t)(micros() - autoRcTimerLoop) >= 0) {

        for (int i = 0; i < 4; i++)
            userRCflag[i] = 0;

        autoRcTimerLoop = micros() + 100000;
    }

}

void resetUser()
{

    rcData[AUX2] = 1200;
}

void unibusAdc1init()
{

    if (adc1Config[UB_ADC_IN1].enabled || adc1Config[UB_ADC_IN2].enabled) {
        ADC_InitTypeDef ADC_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;

        uint8_t adcChannelCount = 0;

        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        if (adc1Config[UB_ADC_IN1].enabled)

        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_Init(GPIOA, &GPIO_InitStructure);

            adc1Config[UB_ADC_IN1].dmaIndex = adcChannelCount;
            adcChannelCount++;
        }

        if (adc1Config[UB_ADC_IN2].enabled)

        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
            GPIO_Init(GPIOA, &GPIO_InitStructure);

            adc1Config[UB_ADC_IN2].dmaIndex = adcChannelCount;
            adcChannelCount++;
        }

        RCC_ADCCLKConfig (RCC_ADC34PLLCLK_Div256); // 72 MHz divided by 256 = 281.25 kHz
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_ADC12, ENABLE);

        DMA_DeInit (DMA1_Channel1);

        DMA_StructInit(&DMA_InitStructure);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & ADC1->DR;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adc1Values;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = adcChannelCount;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc =
                adcChannelCount > 1 ?
                        DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
        DMA_InitStructure.DMA_PeripheralDataSize =
                DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

        DMA_Init(DMA1_Channel1, &DMA_InitStructure);

        DMA_Cmd(DMA1_Channel1, ENABLE);

        // calibrate

        ADC_VoltageRegulatorCmd(ADC1, ENABLE);
        delay(10);
        ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
        ADC_StartCalibration (ADC1);
        while (ADC_GetCalibrationStatus(ADC1) != RESET)
            ;
        ADC_VoltageRegulatorCmd(ADC1, DISABLE);

        ADC_CommonInitTypeDef ADC_CommonInitStructure;

        ADC_CommonStructInit(&ADC_CommonInitStructure);
        ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;
        ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
        ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
        ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
        ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

        ADC_StructInit(&ADC_InitStructure);

        ADC_InitStructure.ADC_ContinuousConvMode =
                ADC_ContinuousConvMode_Enable;
        ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
        ADC_InitStructure.ADC_ExternalTrigConvEvent =
                ADC_ExternalTrigConvEvent_0;
        ADC_InitStructure.ADC_ExternalTrigEventEdge =
                ADC_ExternalTrigEventEdge_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
        ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
        ADC_InitStructure.ADC_NbrOfRegChannel = adcChannelCount;

        ADC_Init(ADC1, &ADC_InitStructure);

        if (adc1Config[UB_ADC_IN1].enabled)
            ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1,
                    ADC_SampleTime_601Cycles5);

        if (adc1Config[UB_ADC_IN2].enabled)
            ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1,
                    ADC_SampleTime_601Cycles5);

        ADC_Cmd(ADC1, ENABLE);

        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY))
            ;

        ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);

        ADC_DMACmd(ADC1, ENABLE);

        ADC_StartConversion(ADC1);

    }

}

void unibusAdc2init()
{

    if (adc2Config[UB_ADC_IN1].enabled || adc2Config[UB_ADC_IN2].enabled) {

        ADC_InitTypeDef ADC_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;

        uint8_t adcChannelCount = 0;

        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        if (adc2Config[UB_ADC_IN1].enabled)

        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
            GPIO_Init(GPIOA, &GPIO_InitStructure);

            adc2Config[UB_ADC_IN1].dmaIndex = adcChannelCount;
            adcChannelCount++;
        }

        if (adc2Config[UB_ADC_IN2].enabled)

        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
            GPIO_Init(GPIOA, &GPIO_InitStructure);

            adc2Config[UB_ADC_IN2].dmaIndex = adcChannelCount;
            adcChannelCount++;
        }

        RCC_ADCCLKConfig (RCC_ADC34PLLCLK_Div256); // 72 MHz divided by 256 = 281.25 kHz
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2 | RCC_AHBPeriph_ADC12, ENABLE);

        DMA_DeInit (DMA2_Channel1);

        DMA_StructInit(&DMA_InitStructure);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & ADC2->DR;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adc2Values;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = adcChannelCount;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc =
                adcChannelCount > 1 ?
                        DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
        DMA_InitStructure.DMA_PeripheralDataSize =
                DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

        DMA_Init(DMA2_Channel1, &DMA_InitStructure);

        DMA_Cmd(DMA2_Channel1, ENABLE);

        // calibrate

        ADC_VoltageRegulatorCmd(ADC2, ENABLE);
        delay(10);
        ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
        ADC_StartCalibration (ADC2);
        while (ADC_GetCalibrationStatus(ADC2) != RESET)
            ;
        ADC_VoltageRegulatorCmd(ADC2, DISABLE);

        ADC_CommonInitTypeDef ADC_CommonInitStructure;

        ADC_CommonStructInit(&ADC_CommonInitStructure);
        ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;
        ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
        ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
        ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
        ADC_CommonInit(ADC2, &ADC_CommonInitStructure);

        ADC_StructInit(&ADC_InitStructure);

        ADC_InitStructure.ADC_ContinuousConvMode =
                ADC_ContinuousConvMode_Enable;
        ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
        ADC_InitStructure.ADC_ExternalTrigConvEvent =
                ADC_ExternalTrigConvEvent_0;
        ADC_InitStructure.ADC_ExternalTrigEventEdge =
                ADC_ExternalTrigEventEdge_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
        ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
        ADC_InitStructure.ADC_NbrOfRegChannel = adcChannelCount;

        ADC_Init(ADC2, &ADC_InitStructure);

        if (adc2Config[UB_ADC_IN1].enabled)
            ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1,
                    ADC_SampleTime_601Cycles5);

        if (adc2Config[UB_ADC_IN2].enabled)
            ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1,
                    ADC_SampleTime_601Cycles5);

        ADC_Cmd(ADC2, ENABLE);

        while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY))
            ;

        ADC_DMAConfig(ADC2, ADC_DMAMode_Circular);

        ADC_DMACmd(ADC2, ENABLE);

        ADC_StartConversion(ADC2);

    }

}

void unibusAdc3init()
{

    if (adc3Config[UB_ADC_IN1].enabled) {

        ADC_InitTypeDef ADC_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;

        uint8_t adcChannelCount = 0;

        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        if (adc3Config[UB_ADC_IN1].enabled)

        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
            GPIO_Init(GPIOB, &GPIO_InitStructure);

            adc3Config[UB_ADC_IN1].dmaIndex = adcChannelCount;
            adcChannelCount++;
        }

        RCC_ADCCLKConfig (RCC_ADC34PLLCLK_Div256); // 72 MHz divided by 256 = 281.25 kHz
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2 | RCC_AHBPeriph_ADC34, ENABLE);

        DMA_DeInit (DMA2_Channel5);

        DMA_StructInit(&DMA_InitStructure);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & ADC3->DR;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adc3Values;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = adcChannelCount;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc =
                adcChannelCount > 1 ?
                        DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
        DMA_InitStructure.DMA_PeripheralDataSize =
                DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

        DMA_Init(DMA2_Channel5, &DMA_InitStructure);

        DMA_Cmd(DMA2_Channel5, ENABLE);

        // calibrate

        ADC_VoltageRegulatorCmd(ADC3, ENABLE);
        delay(10);
        ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
        ADC_StartCalibration (ADC3);
        while (ADC_GetCalibrationStatus(ADC3) != RESET)
            ;
        ADC_VoltageRegulatorCmd(ADC3, DISABLE);

        ADC_CommonInitTypeDef ADC_CommonInitStructure;

        ADC_CommonStructInit(&ADC_CommonInitStructure);
        ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;
        ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
        ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
        ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
        ADC_CommonInit(ADC3, &ADC_CommonInitStructure);

        ADC_StructInit(&ADC_InitStructure);

        ADC_InitStructure.ADC_ContinuousConvMode =
                ADC_ContinuousConvMode_Enable;
        ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
        ADC_InitStructure.ADC_ExternalTrigConvEvent =
                ADC_ExternalTrigConvEvent_0;
        ADC_InitStructure.ADC_ExternalTrigEventEdge =
                ADC_ExternalTrigEventEdge_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
        ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
        ADC_InitStructure.ADC_NbrOfRegChannel = adcChannelCount;

        ADC_Init(ADC3, &ADC_InitStructure);

        if (adc3Config[UB_ADC_IN1].enabled)
            ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 1,
                    ADC_SampleTime_601Cycles5);

        ADC_Cmd(ADC3, ENABLE);

        while (!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY))
            ;

        ADC_DMAConfig(ADC3, ADC_DMAMode_Circular);

        ADC_DMACmd(ADC3, ENABLE);

        ADC_StartConversion(ADC3);

    }

}

void unibusAdc4init()
{

    if (adc4Config[UB_ADC_IN1].enabled || adc4Config[UB_ADC_IN2].enabled
            || adc4Config[UB_ADC_IN3].enabled) {

        ADC_InitTypeDef ADC_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;

        uint8_t adcChannelCount = 0;

        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        if (adc4Config[UB_ADC_IN1].enabled)

        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
            GPIO_Init(GPIOB, &GPIO_InitStructure);

            adc4Config[UB_ADC_IN1].dmaIndex = adcChannelCount;
            adcChannelCount++;
        }

        if (adc4Config[UB_ADC_IN2].enabled)

        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
            GPIO_Init(GPIOB, &GPIO_InitStructure);

            adc4Config[UB_ADC_IN2].dmaIndex = adcChannelCount;
            adcChannelCount++;
        }

        if (adc4Config[UB_ADC_IN3].enabled)

        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
            GPIO_Init(GPIOB, &GPIO_InitStructure);

            adc4Config[UB_ADC_IN3].dmaIndex = adcChannelCount;
            adcChannelCount++;
        }

        RCC_ADCCLKConfig (RCC_ADC34PLLCLK_Div256); // 72 MHz divided by 256 = 281.25 kHz
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2 | RCC_AHBPeriph_ADC34, ENABLE);

        DMA_DeInit (DMA2_Channel2);

        DMA_StructInit(&DMA_InitStructure);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & ADC4->DR;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adc4Values;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = adcChannelCount;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc =
                adcChannelCount > 1 ?
                        DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
        ;
        DMA_InitStructure.DMA_PeripheralDataSize =
                DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

        DMA_Init(DMA2_Channel2, &DMA_InitStructure);

        DMA_Cmd(DMA2_Channel2, ENABLE);

        // calibrate

        ADC_VoltageRegulatorCmd(ADC4, ENABLE);
        delay(10);
        ADC_SelectCalibrationMode(ADC4, ADC_CalibrationMode_Single);
        ADC_StartCalibration (ADC4);
        while (ADC_GetCalibrationStatus(ADC4) != RESET)
            ;
        ADC_VoltageRegulatorCmd(ADC4, DISABLE);

        ADC_CommonInitTypeDef ADC_CommonInitStructure;

        ADC_CommonStructInit(&ADC_CommonInitStructure);
        ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv4;
        ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
        ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
        ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
        ADC_CommonInit(ADC4, &ADC_CommonInitStructure);

        ADC_StructInit(&ADC_InitStructure);

        ADC_InitStructure.ADC_ContinuousConvMode =
                ADC_ContinuousConvMode_Enable;
        ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
        ADC_InitStructure.ADC_ExternalTrigConvEvent =
                ADC_ExternalTrigConvEvent_0;
        ADC_InitStructure.ADC_ExternalTrigEventEdge =
                ADC_ExternalTrigEventEdge_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
        ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
        ADC_InitStructure.ADC_NbrOfRegChannel = adcChannelCount;

        ADC_Init(ADC4, &ADC_InitStructure);

        if (adc4Config[UB_ADC_IN1].enabled)
            ADC_RegularChannelConfig(ADC4, ADC_Channel_3, 1,
                    ADC_SampleTime_601Cycles5);

        if (adc4Config[UB_ADC_IN2].enabled)
            ADC_RegularChannelConfig(ADC4, ADC_Channel_4, 1,
                    ADC_SampleTime_601Cycles5);

        if (adc4Config[UB_ADC_IN3].enabled)
            ADC_RegularChannelConfig(ADC4, ADC_Channel_5, 1,
                    ADC_SampleTime_601Cycles5);

        ADC_Cmd(ADC4, ENABLE);

        while (!ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY))
            ;

        ADC_DMAConfig(ADC4, ADC_DMAMode_Circular);

        ADC_DMACmd(ADC4, ENABLE);

        ADC_StartConversion(ADC4);

    }

}

void unibusAdcConfig()
{

    memset(&adc1Config, 0, sizeof(adc1Config));
    memset(&adc2Config, 0, sizeof(adc2Config));
    memset(&adc3Config, 0, sizeof(adc3Config));
    memset(&adc4Config, 0, sizeof(adc4Config));

}

void unibusAdcInit()
{

    unibusAdc1init();
    unibusAdc2init();
    unibusAdc3init();
    unibusAdc4init();

}

//Extracting GPIO port from unibus pin number
int getGPIOport(unibus_e pin)
{
    int x = 99;
    switch (pin) {
    case Pin2:
    case Pin3:
    case Pin8:
    case Pin10:
    case Pin13:
    case Pin18:
    case Pin19:
        x = 1; //GPIOA
        break;

    case Pin4:
    case Pin5:
    case Pin6:
    case Pin7:
    case Pin9:
    case Pin12:
    case Pin14:
    case Pin15:
    case Pin16:
    case Pin17:
        x = 2; //GPIOB
        break;

    case Pin1:
    case Pin11:
    case Pin20:
        //shouldn't be allowed by IDE
        break;
    }
    return x;
}

GPIO_Pin getGPIOpin(unibus_e pin)
{
    GPIO_Pin x;
    switch (pin) {
    case Pin12:
        x = Pin_0;
        break;

    case Pin19:
        x = Pin_2;
        break;

    case Pin9:
    case Pin18:
        x = Pin_3;
        break;

    case Pin13:
        x = Pin_4;
        break;

    case Pin8:
        x = Pin_5;
        break;

    case Pin7:
    case Pin10:
        x = Pin_8;
        break;

    case Pin6:
        x = Pin_9;
        break;

    case Pin4:
        x = Pin_10;
        break;

    case Pin5:
        x = Pin_11;
        break;

    case Pin14:
        x = Pin_12;
        break;

    case Pin2:
    case Pin15:
        x = Pin_13;
        break;

    case Pin3:
    case Pin16:
        x = Pin_14;
        break;

    case Pin17:
        x = Pin_15;
        break;

    case Pin1:
    case Pin11:
    case Pin20:
        //shouldn't be allowed by IDE
        break;
    }
    return x;
}

uint32_t getGPIOclock(unibus_e pin)
{
    uint32_t x = 99;
    switch (pin) {
    case Pin2:
    case Pin3:
    case Pin8:
    case Pin10:
    case Pin13:
    case Pin18:
    case Pin19:
        x = RCC_AHBPeriph_GPIOA;
        break;

    case Pin4:
    case Pin5:
    case Pin6:
    case Pin7:
    case Pin9:
    case Pin12:
    case Pin14:
    case Pin15:
    case Pin16:
    case Pin17:
        x = RCC_AHBPeriph_GPIOB;
        break;

    case Pin1:
    case Pin11:
    case Pin20:
        //shouldn't be allowed by IDE
        break;
    }
    return x;
}

uint8_t getGPIOpinSource(unibus_e pin)
{
    uint8_t x = 99;
    switch (pin) {
    case Pin12:
        x = GPIO_PinSource0;
        break;

    case Pin19:
        x = GPIO_PinSource2;
        break;

    case Pin9:
    case Pin18:
        x = GPIO_PinSource3;
        break;

    case Pin13:
        x = GPIO_PinSource4;
        break;

    case Pin8:
        x = GPIO_PinSource5;
        break;

    case Pin7:
    case Pin10:
        x = GPIO_PinSource8;
        break;

    case Pin6:
        x = GPIO_PinSource9;
        break;

    case Pin4:
        x = GPIO_PinSource10;
        break;

    case Pin5:
        x = GPIO_PinSource11;
        break;

    case Pin14:
        x = GPIO_PinSource12;
        break;

    case Pin2:
    case Pin15:
        x = GPIO_PinSource13;
        break;

    case Pin3:
    case Pin16:
        x = GPIO_PinSource14;
        break;

    case Pin17:
        x = GPIO_PinSource15;
        break;

    case Pin1:
    case Pin11:
    case Pin20:
        //shouldn't be allowed by IDE
        break;
    }
    return x;
}

uint16_t getTimerCh(unibus_e pin)
{
    uint16_t temp = 99;
    switch (pin) {
    case Pin8:
    case Pin10:
        temp = TIM_Channel_1;
        break;

    case Pin3:
    case Pin9:
    case Pin13:
        temp = TIM_Channel_2;
        break;

    case Pin2:
    case Pin4:
    case Pin12:
    case Pin19:
        temp = TIM_Channel_3;
        break;

    case Pin5:
    case Pin18:
        temp = TIM_Channel_4;
        break;

    case Pin1:
    case Pin6:
    case Pin7:
    case Pin11:
    case Pin14:
    case Pin15:
    case Pin16:
    case Pin17:
    case Pin20:
        //shouldn't be allowed by IDE
        break;
    }
    return temp;
}

uint8_t getADCCh(unibus_e pin)
{
    uint8_t temp = 99;
    switch (pin) {
    case Pin13:
        temp = ADC_Channel_1;
        break;

    case Pin8:
        temp = ADC_Channel_2;
        break;

    case Pin14:
    case Pin19:
        temp = ADC_Channel_3;
        break;

    case Pin16:
    case Pin18:
        temp = ADC_Channel_4;
        break;

    case Pin15:
    case Pin17:
        temp = ADC_Channel_5;
        break;

    case Pin12:
        temp = ADC_Channel_12;
        break;

    case Pin1:
    case Pin2:
    case Pin3:
    case Pin4:
    case Pin5:
    case Pin6:
    case Pin7:
    case Pin9:
    case Pin10:
    case Pin11:
    case Pin20:
        //shouldn't be allowed by IDE
        break;
    }
    return temp;
}

void reverseMotorGPIOInit()
{

    // GPIO setup for reversible motors

    GPIO_TypeDef* gpio;
    gpio_config_t cfg;

    gpio = GPIOB;

    cfg.pin = Pin_4;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    gpioInit(gpio, &cfg);
    digitalLo(GPIOB, Pin_4);

    gpio = GPIOA;

    cfg.pin = Pin_7;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    gpioInit(gpio, &cfg);
    digitalLo(GPIOA, Pin_7);

    gpio = GPIOB;

    cfg.pin = Pin_1;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    gpioInit(gpio, &cfg);
    digitalLo(GPIOB, Pin_1);

    gpio = GPIOA;

    cfg.pin = Pin_15;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    gpioInit(gpio, &cfg);
    digitalLo(GPIOA, Pin_15);

}

