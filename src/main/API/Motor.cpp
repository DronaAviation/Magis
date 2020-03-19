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
#include "drivers/timer.h"
#include "drivers/timer_stm32f30x.h"
#include "drivers/timer_impl.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/ranging_vl53l0x.h"
#include "drivers/opticflow_paw3903.h"
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

#include "Motor.h"
#include "API-Utils.h"

pwmOutputPort_t* userMotor[8];

void Motor_P::init(motor_e motor)
{

    timerHardware_t timerHardware;

    switch (motor) {

    case M1:

        if (!initInternalMotors && !isUserMotorInit[0]) {

#if defined(PRIMUSX)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

            timerHardware = {TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2};
#endif

#if defined(PRIMUSX2)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware ={TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_1};
#endif


            GPIO_PinAFConfig(timerHardware.gpio,
                    (uint16_t) timerHardware.gpioPinSource,
                    timerHardware.alternateFunction);

            uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

            userMotor[0] = pwmOutConfig(&timerHardware, PWM_BRUSHED_TIMER_MHZ,
                    hz / masterConfig.motor_pwm_rate, 0);

            isUserMotorInit[0] = true;
        }

        break;

    case M2:

        if (!initInternalMotors && !isUserMotorInit[1]) {


#if defined(PRIMUX)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource5, GPIO_AF_2};
#endif

#if defined(PRIMUSX2)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1};
#endif

            GPIO_PinAFConfig(timerHardware.gpio,
                    (uint16_t) timerHardware.gpioPinSource,
                    timerHardware.alternateFunction);

            uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

            userMotor[1] = pwmOutConfig(&timerHardware, PWM_BRUSHED_TIMER_MHZ,
                    hz / masterConfig.motor_pwm_rate, 0);

            isUserMotorInit[1] = true;
        }

        break;

    case M3:

        if (!initInternalMotors && !isUserMotorInit[2]) {

#if defined(PRIMUX)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

            timerHardware = {TIM3, GPIOB, Pin_7, TIM_Channel_4, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_10};
#endif

#if defined(PRIMUSX2)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1};
#endif

            GPIO_PinAFConfig(timerHardware.gpio,
                    (uint16_t) timerHardware.gpioPinSource,
                    timerHardware.alternateFunction);

            uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

            userMotor[2] = pwmOutConfig(&timerHardware, PWM_BRUSHED_TIMER_MHZ,
                    hz / masterConfig.motor_pwm_rate, 0);

            isUserMotorInit[2] = true;
        }

        break;

    case M4:

        if (!initInternalMotors && !isUserMotorInit[3]) {

#if defined(PRIMUX)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

            timerHardware = {TIM8, GPIOB, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_5};
#endif

#if defined(PRIMUSX2)

            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB2PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1};
#endif


            GPIO_PinAFConfig(timerHardware.gpio,
                    (uint16_t) timerHardware.gpioPinSource,
                    timerHardware.alternateFunction);

            uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

            userMotor[3] = pwmOutConfig(&timerHardware, PWM_BRUSHED_TIMER_MHZ,
                    hz / masterConfig.motor_pwm_rate, 0);

            isUserMotorInit[3] = true;
        }

        break;

    case M5:

        if (initInternalMotors && !isUserMotorInit[4]) {

#if defined(PRIMUX)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_1};
#endif

#if defined(PRIMUSX2)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

            timerHardware =  {TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2};
#endif


            GPIO_PinAFConfig(timerHardware.gpio,
                    (uint16_t) timerHardware.gpioPinSource,
                    timerHardware.alternateFunction);

            uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

            userMotor[4] = pwmOutConfig(&timerHardware, PWM_BRUSHED_TIMER_MHZ,
                    hz / masterConfig.motor_pwm_rate, 0);

            isUserMotorInit[4] = true;
        }

        break;

    case M6:

        if (initInternalMotors && !isUserMotorInit[5]) {

#if defined(PRIMUX)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1};
#endif

#if defined(PRIMUSX2)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

            timerHardware = {TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_2};
#endif



            GPIO_PinAFConfig(timerHardware.gpio,
                    (uint16_t) timerHardware.gpioPinSource,
                    timerHardware.alternateFunction);

            uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

            userMotor[5] = pwmOutConfig(&timerHardware, PWM_BRUSHED_TIMER_MHZ,
                    hz / masterConfig.motor_pwm_rate, 0);

            isUserMotorInit[5] = true;
        }

        break;

    case M7:

        if (initInternalMotors && !isUserMotorInit[6]) {

#if defined(PRIMUX)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1};
#endif

#if defined(PRIMUSX2)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

            timerHardware ={TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_2};
#endif


            GPIO_PinAFConfig(timerHardware.gpio,
                    (uint16_t) timerHardware.gpioPinSource,
                    timerHardware.alternateFunction);

            uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

            userMotor[6] = pwmOutConfig(&timerHardware, PWM_BRUSHED_TIMER_MHZ,
                    hz / masterConfig.motor_pwm_rate, 0);

            isUserMotorInit[6] = true;
        }

        break;

    case M8:

        if (initInternalMotors && !isUserMotorInit[7]) {

#if defined(PRIMUX)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

            timerHardware = {TIM2, GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1};
#endif

#if defined(PRIMUSX2)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

            timerHardware = {TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_2};
#endif

            GPIO_PinAFConfig(timerHardware.gpio,
                    (uint16_t) timerHardware.gpioPinSource,
                    timerHardware.alternateFunction);

            uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

            userMotor[7] = pwmOutConfig(&timerHardware, PWM_BRUSHED_TIMER_MHZ,
                    hz / masterConfig.motor_pwm_rate, 0);

            isUserMotorInit[7] = true;
        }

        break;

    default:
        break;

    }

}

void Motor_P::initReversibleMotors(void)
{

    initInternalMotors = true;

}

void Motor_P::set(motor_e motor, int16_t pwmValue)
{

    pwmValue = constrain(pwmValue, 1000, 2000);

    switch (motor) {
    case M1:

        if (isUserMotorInit[0]) {

            *(userMotor[0]->ccr) = (pwmValue - 1000) * userMotor[0]->period / 1000;

        } else if (initInternalMotors) {

            motor_disarmed[0] = pwmValue;
        }

        break;

    case M2:

        if (isUserMotorInit[1]) {

            *userMotor[1]->ccr = (pwmValue - 1000) * userMotor[1]->period / 1000;

        } else if (initInternalMotors) {

            motor_disarmed[1] = pwmValue;
        }

        break;

    case M3:

        if (isUserMotorInit[2]) {

            *userMotor[2]->ccr = (pwmValue - 1000) * userMotor[2]->period / 1000;

        } else if (initInternalMotors) {

            motor_disarmed[2] = pwmValue;
        }

        break;

    case M4:

        if (isUserMotorInit[3]) {

            *userMotor[3]->ccr = (pwmValue - 1000) * userMotor[3]->period / 1000;

        } else if (initInternalMotors) {

            motor_disarmed[3] = pwmValue;
        }

        break;

    case M5:

        if (isUserMotorInit[4]) {

            *userMotor[4]->ccr = (pwmValue - 1000) * userMotor[4]->period / 1000;

        } else if (!initInternalMotors) {

            motor_disarmed[0] = pwmValue;
        }

        break;

    case M6:

        if (isUserMotorInit[5]) {

            *userMotor[5]->ccr = (pwmValue - 1000) * userMotor[5]->period / 1000;

        } else if (!initInternalMotors) {

            motor_disarmed[1] = pwmValue;
        }

        break;

    case M7:

        if (isUserMotorInit[6]) {

            *userMotor[6]->ccr = (pwmValue - 1000) * userMotor[6]->period / 1000;

        } else if (!initInternalMotors) {

            motor_disarmed[2] = pwmValue;
        }

        break;

    case M8:

        if (isUserMotorInit[7]) {

            *userMotor[7]->ccr = (pwmValue - 1000) * userMotor[7]->period / 1000;

        } else if (!initInternalMotors) {

            motor_disarmed[3] = pwmValue;
        }

        break;

    default:
        break;

    }

}

void Motor_P::setDirection(motor_e motor, motor_aerial_direction_e direction)
{

    switch (motor) {
    case M1:

        setM1GPIO((bool) direction);
        break;

    case M2:

        setM2GPIO((bool) direction);
        break;

    case M3:

        setM3GPIO((bool) direction);
        break;

    case M4:

        setM4GPIO((bool) direction);
        break;

    default:

        break;

    }

}

void Motor_P::setDirection(motor_e motor,
        motor_terrestrial_direction_e direction)
{

    bool terrestrial_direction;

    if (motor == M1 || motor == M2) {

        if (direction)
            terrestrial_direction = (bool) CLOCK_WISE;
        else
            terrestrial_direction = (bool) ANTICLOCK_WISE;

    } else {

        if (direction)
            terrestrial_direction = (bool) ANTICLOCK_WISE;
        else
            terrestrial_direction = (bool) CLOCK_WISE;
    }

    switch (motor) {
    case M1:

        setM1GPIO(terrestrial_direction);
        break;

    case M2:

        setM2GPIO(terrestrial_direction);
        break;

    case M3:

        setM3GPIO(terrestrial_direction);
        break;

    case M4:

        setM4GPIO(terrestrial_direction);
        break;

    default:

        break;

    }

}

Motor_P Motor;
