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

#if defined(PRIMUSX)

#include "Peripheral.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"
#include "build_config.h"
#include "common/utils.h"
#include "common/atomic.h"

#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "drivers/timer_stm32f30x.h"
#include "drivers/timer_impl.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "sensors/sensors.h"

#include "io/serial.h"
#include "io/display.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/gps_conversion.h"
#include "flight/navigation.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "User.h"
#include "Control.h"
#include "XRanging.h"
#include "Utils.h"

#include "API-Utils.h"

bool gpioReset = false;
bool changeAdress = false;

pwmOutputPort_t* pwm[11];

void GPIO_P::init(unibus_e pin_number, GPIO_Mode_e mode)
{

    uint32_t clock;
    GPIO_TypeDef* gpio;
    gpio_config_t cfg;

    switch(pin_number) {

        case Pin2:

        gpio=GPIOA;
        cfg.pin = Pin_13;
        clock = RCC_AHBPeriph_GPIOA;

        break;

        case Pin3:

        gpio=GPIOA;
        cfg.pin = Pin_14;
        clock = RCC_AHBPeriph_GPIOA;

        break;

        case Pin6:

        gpio=GPIOB;
        cfg.pin = Pin_9;
        clock = RCC_AHBPeriph_GPIOB;

        break;

        case Pin7:

        gpio=GPIOB;
        cfg.pin = Pin_8;
        clock = RCC_AHBPeriph_GPIOB;

        break;

        case Pin8:

        gpio=GPIOA;
        cfg.pin = Pin_5;
        clock = RCC_AHBPeriph_GPIOA;

        break;

        case Pin9:

        gpio=GPIOB;
        cfg.pin = Pin_3;
        clock = RCC_AHBPeriph_GPIOB;

        break;

        case Pin10:

        gpio=GPIOA;
        cfg.pin = Pin_8;
        clock = RCC_AHBPeriph_GPIOA;

        break;

        case Pin13:

        gpio=GPIOA;
        cfg.pin = Pin_4;
        clock = RCC_AHBPeriph_GPIOA;

        break;

        case Pin14:

        gpio=GPIOB;
        cfg.pin = Pin_12;
        clock = RCC_AHBPeriph_GPIOB;

        break;

        case Pin15:

        gpio=GPIOB;
        cfg.pin = Pin_13;
        clock = RCC_AHBPeriph_GPIOB;

        break;

        case Pin16:

        gpio=GPIOB;
        cfg.pin = Pin_14;
        clock = RCC_AHBPeriph_GPIOB;

        break;

        case Pin17:

        gpio=GPIOB;
        cfg.pin = Pin_15;
        clock = RCC_AHBPeriph_GPIOB;

        break;

        case Pin18:

        gpio=GPIOA;
        cfg.pin = Pin_3;
        clock = RCC_AHBPeriph_GPIOA;

        break;

        case Pin19:

        gpio=GPIOA;
        cfg.pin = Pin_2;
        clock = RCC_AHBPeriph_GPIOA;

        break;

    }

    switch (mode) {
        case INPUT:
        cfg.mode = Mode_IN_FLOATING;
        break;

        case INPUT_PU:
        cfg.mode = Mode_IPU;
        break;

        case INPUT_PD:
        cfg.mode = Mode_IPD;
        break;

        case OUTPUT:
        cfg.mode = Mode_Out_PP;
        break;

    }

    cfg.speed = Speed_2MHz;

    RCC_AHBPeriphClockCmd(clock, ENABLE);

    gpioInit(gpio, &cfg);
}

bool GPIO_P::read(unibus_e pin_number)
{

    uint16_t pin;
    GPIO_TypeDef* gpio;

    switch(pin_number) {

        case Pin2:

        gpio=GPIOA;
        pin = Pin_13;

        break;

        case Pin3:

        gpio=GPIOA;
        pin = Pin_14;

        break;

        case Pin6:

        gpio=GPIOB;
        pin = Pin_9;

        break;

        case Pin7:

        gpio=GPIOB;
        pin = Pin_8;

        break;

        case Pin8:

        gpio=GPIOA;
        pin = Pin_5;

        break;

        case Pin9:

        gpio=GPIOB;
        pin = Pin_3;

        break;

        case Pin10:

        gpio=GPIOA;
        pin = Pin_8;

        break;

        case Pin13:

        gpio=GPIOA;
        pin = Pin_4;

        break;

        case Pin14:

        gpio=GPIOB;
        pin = Pin_12;

        break;

        case Pin15:

        gpio=GPIOB;
        pin = Pin_13;

        break;

        case Pin16:

        gpio=GPIOB;
        pin = Pin_14;

        break;

        case Pin17:

        gpio=GPIOB;
        pin = Pin_15;

        break;

        case Pin18:

        gpio=GPIOA;
        pin = Pin_3;

        break;

        case Pin19:

        gpio=GPIOA;
        pin = Pin_2;

        break;

    }

    return digitalIn(gpio, pin);

}

void GPIO_P::write(unibus_e pin_number, GPIO_State_e STATE)
{

    uint16_t pin;
    GPIO_TypeDef* gpio;

    switch(pin_number) {

        case Pin2:

        gpio=GPIOA;
        pin = Pin_13;

        break;

        case Pin3:

        gpio=GPIOA;
        pin = Pin_14;

        break;

        case Pin6:

        gpio=GPIOB;
        pin = Pin_9;

        break;

        case Pin7:

        gpio=GPIOB;
        pin = Pin_8;

        break;

        case Pin8:

        gpio=GPIOA;
        pin = Pin_5;

        break;

        case Pin9:

        gpio=GPIOB;
        pin = Pin_3;

        break;

        case Pin10:

        gpio=GPIOA;
        pin = Pin_8;

        break;

        case Pin13:

        gpio=GPIOA;
        pin = Pin_4;

        break;

        case Pin14:

        gpio=GPIOB;
        pin = Pin_12;

        break;

        case Pin15:

        gpio=GPIOB;
        pin = Pin_13;

        break;

        case Pin16:

        gpio=GPIOB;
        pin = Pin_14;

        break;

        case Pin17:

        gpio=GPIOB;
        pin = Pin_15;

        break;

        case Pin18:

        gpio=GPIOA;
        pin = Pin_3;

        break;

        case Pin19:

        gpio=GPIOA;
        pin = Pin_2;

        break;

    }

    switch(STATE) {

        case STATE_LOW:

        digitalLo(gpio, pin);
        break;

        case STATE_HIGH:

        digitalHi(gpio, pin);
        break;

        case STATE_TOGGLE:

        digitalToggle(gpio, pin);
        break;
    }
}

void ADC_P::init(unibus_e pin_number)
{

    switch (pin_number) {

        case Pin8:

        isADCEnable[UB_ADC2_IN2]= true;

        break;

        case Pin13:

        isADCEnable[UB_ADC2_IN1]= true;
        break;

        case Pin14:

        isADCEnable[UB_ADC4_IN3]= true;
        break;

        case Pin15:

        isADCEnable[UB_ADC3_IN5]= true;
        break;

        case Pin16:

        isADCEnable[UB_ADC4_IN4]= true;
        break;

        case Pin17:

        isADCEnable[UB_ADC4_IN5]= true;
        break;

        case Pin18:

        isADCEnable[UB_ADC1_IN4]= true;
        break;

        case Pin19:

        isADCEnable[UB_ADC1_IN3]= true;
        break;

        default:

        break;

    }

}

uint16_t ADC_P::read(unibus_e pin_number)
{

    switch (pin_number) {

        case Pin8:

        return adc2Values[adcDmaIndex[UB_ADC2_IN2]];

        break;

        case Pin13:

        return adc2Values[adcDmaIndex[UB_ADC2_IN1]];
        break;

        case Pin14:

        return adc4Values[adcDmaIndex[UB_ADC4_IN3]];
        break;

        case Pin15:

        return adc3Values[adcDmaIndex[UB_ADC3_IN5]];
        break;

        case Pin16:

        return adc4Values[adcDmaIndex[UB_ADC4_IN4]];
        break;

        case Pin17:

        return adc4Values[adcDmaIndex[UB_ADC4_IN5]];
        break;

        case Pin18:

        return adc1Values[adcDmaIndex[UB_ADC1_IN4]];
        break;

        case Pin19:

        return adc1Values[adcDmaIndex[UB_ADC1_IN3]];
        break;

        default:

        return 0;
        break;

    }

}

static serialPort_t* uart2;
static serialPort_t* uart3;

portMode_t mapped_mode;
portOptions_t mapped_options;

uint32_t getBaud(UART_Baud_Rate_e BAUD)
{
    switch (BAUD) {

        case BAUD_RATE_4800:

        return 4800;

        break;

        case BAUD_RATE_9600:

        return 9600;

        break;

        case BAUD_RATE_14400:

        return 14400;

        break;

        case BAUD_RATE_19200:

        return 19200;

        break;

        case BAUD_RATE_38400:

        return 38400;

        break;

        case BAUD_RATE_57600:

        return 57600;

        break;

        case BAUD_RATE_115200:

        return 115200;

        break;

        case BAUD_RATE_128000:

        return 128000;

        break;

        case BAUD_RATE_256000:

        return 256000;

        break;
    }
}

void UART_P::init(UART_Port_e PORT, UART_Baud_Rate_e BAUD)
{

    if(PORT==UART2) {

        uart2 = openSerialPort(SERIAL_PORT_USART2, FUNCTION_UNIBUS, NULL,
                getBaud(BAUD), MODE_RXTX, SERIAL_NOT_INVERTED);

    } else if(PORT==UART3) {

        uart3 = openSerialPort(SERIAL_PORT_USART3, FUNCTION_UNIBUS, NULL,
                getBaud(BAUD), MODE_RXTX, SERIAL_NOT_INVERTED);

    }
}

uint8_t UART_P::read8(UART_Port_e PORT)
{

    if(PORT==UART2) {

        return serialRead(uart2) & 0xff;

    } else if(PORT==UART3) {

        return serialRead(uart3) & 0xff;

    }
}

uint16_t UART_P::read16(UART_Port_e PORT)
{

    uint16_t t = read8(PORT);

    t += (uint16_t) read8(PORT) << 8;

    return t;
}

uint32_t UART_P::read32(UART_Port_e PORT)
{

    uint32_t t = read16(PORT);

    t += (uint32_t) read16(PORT) << 16;

    return t;

}

void UART_P::write(UART_Port_e PORT, uint8_t data)
{

    if(PORT==UART2) {

        serialWrite(uart2, data);

    } else if(PORT==UART3) {

        serialWrite(uart3, data);

    }

}

void UART_P::write(UART_Port_e PORT, const char *str)
{

    if(PORT==UART2) {

        serialPrint(uart2, str);

    } else if(PORT==UART3) {

        serialPrint(uart3, str);

    }

}

void UART_P::write(UART_Port_e PORT, uint8_t* data, uint16_t length)
{

    if(PORT==UART2) {

        while (length--) {

            serialWrite(uart2, *data);
            data++;
        }

    } else if(PORT==UART3) {

        while (length--) {

            serialWrite(uart3, *data);
            data++;
        }

    }

}

bool UART_P::rxBytesWaiting(UART_Port_e PORT)
{

    if(PORT==UART2) {

        return serialRxBytesWaiting(uart2);

    } else if(PORT==UART3) {

        return serialRxBytesWaiting(uart3);

    }
}

bool UART_P::txBytesFree(UART_Port_e PORT)
{

    if(PORT==UART2) {

        return serialTxBytesFree(uart2);

    } else if(PORT==UART3) {

        return serialTxBytesFree(uart3);
    }
}

uint8_t* I2C_P::read(uint8_t device_add, uint8_t reg, uint32_t length)
{
    uint8_t buffer[length];

    i2cRead(device_add, reg, length, (uint8_t *)buffer);

    return buffer;
}

bool I2C_P::write(uint8_t device_add, uint8_t reg, uint32_t length, uint8_t* data)
{
    bool status;
    status=i2cWriteBuffer(device_add, reg, length, data);
    return status;
}

void PWM_P::init(unibus_e pin_number, uint16_t pwmRate) {

    timerHardware_t timerHardware;
    uint32_t hz;

    switch (pin_number) {

        case Pin2:

        if(!isPwmInit[0]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

            timerHardware = {TIM4, GPIOA, Pin_13, TIM_Channel_3,TIM4_IRQn, 1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_10};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[0] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[0]=true;

        }

        break;

        case Pin3:

        if(!isPwmInit[1]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

            timerHardware = {TIM8, GPIOA, Pin_14, TIM_Channel_2,TIM8_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_5};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[1] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[1]=true;

        }

        break;

        case Pin4:

        if(!isPwmInit[2]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[2] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[2]=true;

        }

        break;

        case Pin5:

        if(!isPwmInit[3]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[3] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[3]=true;

        }

        break;

        case Pin8:

        if(!isPwmInit[4]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOA, Pin_5, TIM_Channel_1, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource5, GPIO_AF_1};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[4] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[4]=true;

        }

        break;

        case Pin9:

        if(!isPwmInit[5]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOB, Pin_3, TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_1};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[5] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[5]=true;

        }

        break;

        case Pin10:

        if(!isPwmInit[6]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

            timerHardware = {TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource8, GPIO_AF_6};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[6] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[6]=true;

        }

        break;

        case Pin13:

        if(!isPwmInit[7]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

            timerHardware = {TIM3, GPIOA, Pin_4, TIM_Channel_2, TIM3_IRQn, 0, Mode_AF_PP, GPIO_PinSource4, GPIO_AF_2};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[7] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[7]=true;

        }

        break;
        /*
         case Pin15:

         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

         RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);



         timerHardware = {TIM1, GPIOB, Pin_13, TIM_Channel_11, TIM1_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_6};

         GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

         hz = PWM_TIMER_MHZ * 1000000;

         pwm[8] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

         isPwmInit[8]=true;

         break;
         */

        case Pin18:

        if(!isPwmInit[9]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_1};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[9] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[9]=true;

        }

        break;

        case Pin19:

        if(!isPwmInit[10]) {

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_1};

            GPIO_PinAFConfig(timerHardware.gpio, (uint16_t) timerHardware.gpioPinSource, timerHardware.alternateFunction);

            hz = PWM_TIMER_MHZ * 1000000;

            pwm[10] = pwmOutConfig(&timerHardware, PWM_TIMER_MHZ, hz / pwmRate, 1500);

            isPwmInit[10]=true;

        }

        break;

    }

}

void PWM_P::write(unibus_e pin_number, uint16_t pwmValue) {

    pwmValue=constrain(pwmValue,500,2500);

    switch (pin_number) {

        case Pin2:

        if(isPwmInit[0]) {

            *pwm[0]->ccr=pwmValue;

        }

        break;

        case Pin3:

        if(isPwmInit[1]) {

            *pwm[1]->ccr=pwmValue;

        }

        break;

        case Pin4:

        if(isPwmInit[2]) {

            *pwm[2]->ccr=pwmValue;

        }

        break;

        case Pin5:

        if(isPwmInit[3]) {

            *pwm[3]->ccr=pwmValue;

        }

        break;

        case Pin8:

        if(isPwmInit[4]) {

            *pwm[4]->ccr=pwmValue;

        }

        break;

        case Pin9:

        if(isPwmInit[5]) {

            *pwm[5]->ccr=pwmValue;

        }

        break;

        case Pin10:

        if(isPwmInit[6]) {

            *pwm[6]->ccr=pwmValue;

        }

        break;

        case Pin13:

        if(isPwmInit[7]) {

            *pwm[7]->ccr=pwmValue;

        }

        break;
        /*
         case Pin15:

         if(isPwmInit[8]){

         *pwm[8]->ccr=pwmValue;

         }

         break;
         */
        case Pin18:

        if(isPwmInit[9]) {

            *pwm[9]->ccr=pwmValue;

        }

        break;

        case Pin19:

        if(isPwmInit[10]) {

            *pwm[10]->ccr=pwmValue;

        }

        break;

        default:

        break;

    }

}

GPIO_P GPIO;
ADC_P ADC;
UART_P UART;
I2C_P I2C;
PWM_P PWM;

#endif
