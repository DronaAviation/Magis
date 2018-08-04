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

#include "Peripheral.h"

#include_next <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"
#include "build_config.h"
#include "common/utils.h"
#include "common/atomic.h"

#include "drivers/nvic.h"
#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "drivers/timer_stm32f30x.h"
#include "drivers/timer_impl.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "../API-Utils.h"
bool gpioReset = false;
bool changeAdress = false;

/*Mapping function*/

uint8_t getGPIOaf(GPIOAF_t AF)
{
    uint8_t af = 0;
    switch (AF) {
    case AF0:
        af = GPIO_AF_0;
        break;

    case AF1:
        af = GPIO_AF_1;
        break;

    case AF2:
        af = GPIO_AF_2;
        break;

    case AF3:
        af = GPIO_AF_3;
        break;

    case AF4:
        af = GPIO_AF_4;
        break;

    case AF5:
        af = GPIO_AF_5;
        break;

    case AF6:
        af = GPIO_AF_6;
        break;

    case AF7:
        af = GPIO_AF_7;
        break;

    case AF8:
        af = GPIO_AF_8;
        break;

    case AF9:
        af = GPIO_AF_9;
        break;

    case AF10:
        af = GPIO_AF_10;
        break;

    case AF11:
        af = GPIO_AF_11;
        break;

    case AF12:
        af = GPIO_AF_12;
        break;

    case AF13:
        af = GPIO_AF_13;
        break;

    case AF14:
        af = GPIO_AF_14;
        break;

    case AF15:
        af = GPIO_AF_15;
        break;
    }
    return af;
}

void GPIOClass::Init(unibus_e pin_number, GPIO_Mode_e mode, GPIO_Speed_e speed)
{
    int gpioX;
    GPIO_Pin pinY;
    uint32_t clockZ;
    gpioX = getGPIOport(pin_number);
    pinY = getGPIOpin(pin_number);
    clockZ = getGPIOclock(pin_number);

    GPIO_TypeDef* gpio;
    gpio_config_t cfg;
    if (gpioX == 1) {
        gpio = GPIOA;
    } else if (gpioX == 2) {
        gpio = GPIOB;
    }

    cfg.pin = pinY;
    switch (mode) {
    case AIN:
        cfg.mode = Mode_AIN;
        break;

    case IN_FLOATING:
        cfg.mode = Mode_IN_FLOATING;
        break;

    case IPD:
        cfg.mode = Mode_IPD;
        break;

    case IPU:
        cfg.mode = Mode_IPU;
        break;

    case Out_OD:
        cfg.mode = Mode_Out_OD;
        break;

    case Out_PP:
        cfg.mode = Mode_Out_PP;
        break;

    case AF_OD:
        cfg.mode = Mode_AF_OD;
        break;

    case AF_PP:
        cfg.mode = Mode_AF_PP;
        break;

    case AF_PP_PD:
        cfg.mode = Mode_AF_PP_PD;
        break;

    case AF_PP_PU:
        cfg.mode = Mode_AF_PP_PU;
        break;
    }
    switch (speed) {
    case Speed_2MHz:
        cfg.speed = Speed_2MHz;
        break;

    case Speed_10MHz:
        cfg.speed = Speed_10MHz;
        break;

    case Speed_50MHz:
        cfg.speed = Speed_50MHz;
        break;
    }

    RCC_AHBPeriphClockCmd(clockZ, ENABLE);

    gpioInit(gpio, &cfg);
}

void GPIOClass::AFConfig(unibus_e pin_number, GPIOAF_t AF)
{
    int gpioX;
    uint8_t pinSrcY;
    uint8_t af;

    gpioX = getGPIOport(pin_number);
    pinSrcY = getGPIOpinSource(pin_number);
    af = getGPIOaf(AF);

    GPIO_TypeDef* gpio;
    if (gpioX == 1) {
        gpio = GPIOA;
    } else if (gpioX == 2) {
        gpio = GPIOB;
    }

    gpioAFConfig(gpio, pinSrcY, af);
}

void GPIOClass::setHigh(unibus_e pin_number)
{

    GPIO_TypeDef* gpio;

    int gpioX = getGPIOport(pin_number);

    if (gpioX == 1) {
        gpio = GPIOA;
    } else if (gpioX == 2) {
        gpio = GPIOB;
    }

    digitalHi(gpio, getGPIOpin(pin_number));

}

void GPIOClass::setLow(unibus_e pin_number)
{

    GPIO_TypeDef* gpio;

    int gpioX = getGPIOport(pin_number);

    if (gpioX == 1) {
        gpio = GPIOA;
    } else if (gpioX == 2) {
        gpio = GPIOB;
    }

    digitalLo(gpio, getGPIOpin(pin_number));

}

bool GPIOClass::read(unibus_e pin_number)
{

    GPIO_TypeDef* gpio;

    int gpioX = getGPIOport(pin_number);

    if (gpioX == 1) {
        gpio = GPIOA;
    } else if (gpioX == 2) {
        gpio = GPIOB;
    }

    return digitalIn(gpio, getGPIOpin(pin_number));

}



class TimerClass::TimerImpl {

public:
    pwmOutputPort_t * pwmOutPort;

};


TimerClass::TimerClass()
{

}


void TimerClass::init(unibus_e pin_number, uint16_t hz)
{

    timerHardware_t timerHardware = { TIM4, GPIOA, Pin_13, TIM_Channel_3,TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_10 };

    GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

    uint32_t ghz = PWM_BRUSHED_TIMER_MHZ * 1000000;

    this->timerimpl_ = new TimerImpl;
    this->timerimpl_->pwmOutPort = pwmOutConfig(&timerHardware, 1, 1000000 / hz, 1000);

}

void TimerClass::setPWM(uint16_t value)
{

    *(this->timerimpl_->pwmOutPort->ccr) = value;

}

void ADCClass::Init(unibus_e pin_number)
{

    switch (pin_number) {

    case Pin8:

        adc2Config[UB_ADC_IN2].enabled = true;

        break;

    case Pin13:

        adc2Config[UB_ADC_IN1].enabled = true;
        break;

    case Pin14:

        adc4Config[UB_ADC_IN1].enabled = true;
        break;

    case Pin15:

        adc3Config[UB_ADC_IN1].enabled = true;
        break;

    case Pin16:

        adc4Config[UB_ADC_IN2].enabled = true;
        break;

    case Pin17:

        adc4Config[UB_ADC_IN3].enabled = true;
        break;

    case Pin18:

        adc1Config[UB_ADC_IN2].enabled = true;
        break;

    case Pin19:

        adc1Config[UB_ADC_IN1].enabled = true;
        break;

    default:
        break;

    }

}

uint16_t ADCClass::Read(unibus_e pin_number)
{

    switch (pin_number) {

    case Pin8:

        return adc2Values[adc2Config[UB_ADC_IN2].dmaIndex];

        break;

    case Pin13:

        return adc2Values[adc2Config[UB_ADC_IN1].dmaIndex];
        break;

    case Pin14:

        return adc4Values[adc4Config[UB_ADC_IN1].dmaIndex];
        break;

    case Pin15:

        return adc3Values[adc3Config[UB_ADC_IN1].dmaIndex];
        break;

    case Pin16:

        return adc4Values[adc4Config[UB_ADC_IN2].dmaIndex];
        break;

    case Pin17:

        return adc4Values[adc4Config[UB_ADC_IN3].dmaIndex];
        break;

    case Pin18:

        return adc1Values[adc1Config[UB_ADC_IN2].dmaIndex];
        break;

    case Pin19:

        return adc1Values[adc1Config[UB_ADC_IN1].dmaIndex];
        break;

    default:

        return 0;
        break;

    }

}

GPIOClass GPIO;
ADCClass ADC;

