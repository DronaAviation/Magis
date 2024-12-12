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

pwmOutputPort_t *userMotor [ 8 ];


void setupMotorAndGPIO ( int index, uint32_t GPIOx_RCC, uint32_t TIMx_RCC, TIM_TypeDef *TIMx, GPIO_TypeDef *GPIOx, uint16_t pin_x, uint8_t channel, uint8_t irqn, GPIO_Mode mode, uint16_t pinSource, uint8_t AF, GPIO_TypeDef *gpio, uint16_t cfgPin, bool digitalState ) {

  timerHardware_t timerHardware;
  gpio_config_t cfg;

  RCC_AHBPeriphClockCmd ( GPIOx_RCC, ENABLE );
  RCC_APB1PeriphClockCmd ( TIMx_RCC, ENABLE );

  timerHardware = { TIMx, GPIOx, pin_x, channel, irqn, 1, mode, pinSource, AF };

  GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

  uint32_t hz         = PWM_BRUSHED_TIMER_MHZ * 1000000;
  userMotor [ index ] = pwmOutConfig ( &timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / masterConfig.motor_pwm_rate, 0 );

  cfg.pin   = cfgPin;
  cfg.mode  = Mode_Out_PP;
  cfg.speed = Speed_2MHz;
  RCC_AHBPeriphClockCmd ( GPIOx_RCC, ENABLE );
  gpioInit ( gpio, &cfg );

  if ( digitalState )
    digitalHi ( gpio, cfgPin );
  else
    digitalLo ( gpio, cfgPin );
}

void Motor_P::initReverseMotor ( reverse_motor_e motor ) {
  switch ( motor ) {
#ifdef PRIMUSX
    case M1:
      setupMotorAndGPIO ( 0, RCC_AHBPeriph_GPIOA, RCC_APB1Periph_TIM3, TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2, GPIOB, Pin_4, false );
      break;
    case M2:
      setupMotorAndGPIO ( 1, RCC_AHBPeriph_GPIOB, RCC_APB1Periph_TIM2, TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, Mode_AF_PP, GPIO_PinSource5, GPIO_AF_2, GPIOA, Pin_7, true );
      break;
    case M3:
      setupMotorAndGPIO ( 2, RCC_AHBPeriph_GPIOB, RCC_APB1Periph_TIM3, TIM3, GPIOB, Pin_7, TIM_Channel_4, TIM3_IRQn, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_10, GPIOB, Pin_1, false );
      break;
    case M4:
      setupMotorAndGPIO ( 3, RCC_AHBPeriph_GPIOB, RCC_APB2Periph_TIM8, TIM8, GPIOB, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_5, GPIOA, Pin_15, true );
      break;
#endif
#ifdef PRIMUSX2
    case M1:
      setupMotorAndGPIO ( 0, RCC_AHBPeriph_GPIOA, RCC_APB1Periph_TIM2, TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_1, GPIOB, Pin_4, false );
      break;
    case M2:
      setupMotorAndGPIO ( 1, RCC_AHBPeriph_GPIOA, RCC_APB1Periph_TIM2, TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1, GPIOB, Pin_5, false );
      break;
    case M3:
      setupMotorAndGPIO ( 2, RCC_AHBPeriph_GPIOB, RCC_APB1Periph_TIM2, TIM2, GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1, GPIOB, Pin_7, false );
      break;
    case M4:
      setupMotorAndGPIO ( 3, RCC_AHBPeriph_GPIOB, RCC_APB1Periph_TIM2, TIM2, GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1, GPIOB, Pin_6, false );
      break;
#endif
    default:
      break;
  }
}

void Motor_P::set ( std_motor_e motor, int16_t pwmValue ) {
  bool morto_arm_stat;
  pwmValue = constrain ( pwmValue, 1000, 2000 );

  if ( ! IS_RC_MODE_ACTIVE ( BOXARM ) ) {
    morto_arm_stat = false;
    switch ( motor ) {

      case M5:
        motor_disarmed [ 3 ] = pwmValue;
        break;

      case M6:
        motor_disarmed [ 2 ] = pwmValue;
        break;

      case M7:
        motor_disarmed [ 0 ] = pwmValue;
        break;

      case M8:
        motor_disarmed [ 1 ] = pwmValue;
        break;

      default:
        break;
    }
  } else if ( IS_RC_MODE_ACTIVE ( BOXARM ) && ! morto_arm_stat ) {
    morto_arm_stat       = true;
    motor_disarmed [ 3 ] = 1000;
    motor_disarmed [ 2 ] = 1000;
    motor_disarmed [ 0 ] = 1000;
    motor_disarmed [ 1 ] = 1000;
  }
}

void Motor_P::set ( reverse_motor_e motor, int16_t pwmValue ) {
  pwmValue = constrain ( pwmValue, 1000, 2000 );

  switch ( motor ) {
    case M1:
      *userMotor [ 0 ]->ccr = ( pwmValue - 1000 ) * userMotor [ 0 ]->period / 1000;
      break;

    case M2:
      *userMotor [ 1 ]->ccr = ( pwmValue - 1000 ) * userMotor [ 1 ]->period / 1000;
      break;

    case M3:
      *userMotor [ 2 ]->ccr = ( pwmValue - 1000 ) * userMotor [ 2 ]->period / 1000;
      break;

    case M4:
      *userMotor [ 3 ]->ccr = ( pwmValue - 1000 ) * userMotor [ 3 ]->period / 1000;
      break;

    default:
      break;
  }
}

void Motor_P::set ( reverse_motor_e motor, motor_direction_e direction, int16_t pwmValue ) {

  pwmValue = constrain ( pwmValue, 1000, 2000 );

  switch ( motor ) {
    case M1:
      if ( direction )
        digitalHi ( GPIOB, Pin_4 );
      else
        digitalLo ( GPIOB, Pin_4 );
      *userMotor [ 0 ]->ccr = ( pwmValue - 1000 ) * userMotor [ 0 ]->period / 1000;
      break;

    case M2:
      if ( direction )
        digitalHi ( GPIOA, Pin_7 );
      else
        digitalLo ( GPIOA, Pin_7 );
      *userMotor [ 1 ]->ccr = ( pwmValue - 1000 ) * userMotor [ 1 ]->period / 1000;
      break;

    case M3:
      if ( direction )
        digitalHi ( GPIOB, Pin_1 );
      else
        digitalLo ( GPIOB, Pin_1 );

      *userMotor [ 2 ]->ccr = ( pwmValue - 1000 ) * userMotor [ 2 ]->period / 1000;
      break;

    case M4:
      if ( direction )
        digitalHi ( GPIOA, Pin_15 );
      else
        digitalLo ( GPIOA, Pin_15 );
      *userMotor [ 3 ]->ccr = ( pwmValue - 1000 ) * userMotor [ 3 ]->period / 1000;
      break;

    default:
      break;
  }
}
struct Rev_Motor_Gpio {
  GPIO_TypeDef *gpio;
  uint16_t pin;
};

// Initialize an array of Motor structs
#if defined( PRIMUSX )
Rev_Motor_Gpio motors_gpio [] = {
  { GPIOB, Pin_4 },
  { GPIOA, Pin_7 },
  { GPIOB, Pin_1 },
  { GPIOA, Pin_15 }
};
#endif
#if defined( PRIMUS2X )
Rev_Motor_Gpio motors_gpio [] = {
  { GPIOB, Pin_4 },
  { GPIOB, Pin_5 },
  { GPIOB, Pin_7 },
  { GPIOB, Pin_6 }
};
#endif

void Motor_P::setDirection ( reverse_motor_e motor, motor_direction_e direction ) {
  // Check if the motor index is within the valid range
  if ( motor >= 0 && motor < sizeof ( motors_gpio ) / sizeof ( Rev_Motor_Gpio ) ) {
    if ( direction )
      digitalHi ( motors_gpio [ motor ].gpio, motors_gpio [ motor ].pin );
    else
      digitalLo ( motors_gpio [ motor ].gpio, motors_gpio [ motor ].pin );
  }
}

Motor_P Motor;
