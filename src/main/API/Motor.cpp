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

// pwmOutputPort_t *userMotor [ 8 ];

// void MotorInit ( int index, uint32_t RCC_AHBPeriph, uint32_t RCC_APB1Periph, TIM_TypeDef *tim, GPIO_TypeDef *gpio, uint16_t pin, uint8_t channel, uint8_t irq, uint8_t outputEnable, GPIO_Mode gpioInputMode, uint8_t gpioPinSource, uint8_t alternateFunction ) {
//   timerHardware_t timerHardware;

//   RCC_AHBPeriphClockCmd ( RCC_AHBPeriph, ENABLE );

//   RCC_APB1PeriphClockCmd ( RCC_APB1Periph, ENABLE );

//   timerHardware = { tim, gpio, pin, channel, irq, outputEnable, gpioInputMode, gpioPinSource, alternateFunction };

//   GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );

//   uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

//   userMotor [ index ] = pwmOutConfig ( &timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / masterConfig.motor_pwm_rate, 0 );
// }

pwmOutputPort_t *userMotor [ 4 ];

struct Rev_Motor_Gpio {
  GPIO_TypeDef *gpio;
  uint16_t pin;
  uint32_t RCC_AHBPeriph;
};

// Initialize an array of Motor structs
Rev_Motor_Gpio motors_gpio [ 4 ];


void Motor_P::initReverseMotor ( reverse_motor_e motor ) {

  timerHardware_t timerHardware;
  GPIO_TypeDef *gpio;
  gpio_config_t cfg;
  uint32_t hz = PWM_BRUSHED_TIMER_MHZ * 1000000;

  switch ( motor ) {

    case M1:
#if defined( PRIMUSX )
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );
      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3, ENABLE );
      timerHardware                   = { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2 };
      motors_gpio [ 0 ].gpio          = GPIOB;
      motors_gpio [ 0 ].pin           = Pin_4;
      motors_gpio [ 0 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;

#endif

#if defined( PRIMUSX2 )
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );
      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2, ENABLE );
      timerHardware                   = { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_1 };
      motors_gpio [ 0 ].gpio          = GPIOB;
      motors_gpio [ 0 ].pin           = Pin_4;
      motors_gpio [ 0 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
#endif
      // M1
      cfg.pin   = motors_gpio [ 0 ].pin;
      cfg.mode  = Mode_Out_PP;
      cfg.speed = Speed_2MHz;
      RCC_AHBPeriphClockCmd ( motors_gpio [ 0 ].RCC_AHBPeriph, ENABLE );
      gpioInit ( motors_gpio [ 0 ].gpio, &cfg );
      digitalLo ( motors_gpio [ 0 ].gpio, motors_gpio [ 0 ].pin );

      GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );
      userMotor [ 0 ] = pwmOutConfig ( &timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / masterConfig.motor_pwm_rate, 0 );
      break;

    case M2:
#if defined( PRIMUSX )
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );
      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2, ENABLE );
      timerHardware                   = { TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource5, GPIO_AF_2 };
      motors_gpio [ 1 ].gpio          = GPIOA;
      motors_gpio [ 1 ].pin           = Pin_7;
      motors_gpio [ 1 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOA;
#endif

#if defined( PRIMUSX2 )
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOA, ENABLE );
      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2, ENABLE );
      timerHardware                   = { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1 };
      motors_gpio [ 1 ].gpio          = GPIOB;
      motors_gpio [ 1 ].pin           = Pin_5;
      motors_gpio [ 1 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB2
#endif
                                                  cfg.pin
                = motors_gpio [ 1 ].pin;
      cfg.mode  = Mode_Out_PP;
      cfg.speed = Speed_2MHz;
      RCC_AHBPeriphClockCmd ( motors_gpio [ 1 ].RCC_AHBPeriph, ENABLE );
      gpioInit ( motors_gpio [ 1 ].gpio, &cfg );
      digitalHi ( motors_gpio [ 1 ].gpio, motors_gpio [ 1 ].pin );

      GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );
      userMotor [ 1 ] = pwmOutConfig ( &timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / masterConfig.motor_pwm_rate, 0 );
      break;

    case M3:
#if defined( PRIMUSX )
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );
      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3, ENABLE );
      timerHardware                   = { TIM3, GPIOB, Pin_7, TIM_Channel_4, TIM3_IRQn, 1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_10 };
      motors_gpio [ 2 ].gpio          = GPIOB;
      motors_gpio [ 2 ].pin           = Pin_1;
      motors_gpio [ 2 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
#endif

#if defined( PRIMUSX2 )
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );
      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2, ENABLE );
      timerHardware                   = { TIM2, GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1 };
      motors_gpio [ 2 ].gpio          = GPIOB;
      motors_gpio [ 2 ].pin           = Pin_7;
      motors_gpio [ 2 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
#endif
      cfg.pin   = motors_gpio [ 2 ].pin;
      cfg.mode  = Mode_Out_PP;
      cfg.speed = Speed_2MHz;
      RCC_AHBPeriphClockCmd ( motors_gpio [ 2 ].RCC_AHBPeriph, ENABLE );
      gpioInit ( motors_gpio [ 2 ].gpio, &cfg );
      digitalLo ( motors_gpio [ 2 ].gpio, motors_gpio [ 2 ].pin );

      GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );
      userMotor [ 2 ] = pwmOutConfig ( &timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / masterConfig.motor_pwm_rate, 0 );
      break;

    case M4:
#if defined( PRIMUSX )
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );
      RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM8, ENABLE );
      timerHardware                   = { TIM8, GPIOB, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_5 };
      motors_gpio [ 3 ].gpio          = GPIOA;
      motors_gpio [ 3 ].pin           = Pin_15;
      motors_gpio [ 3 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
#endif

#if defined( PRIMUSX2 )
      RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );
      RCC_APB2PeriphClockCmd ( RCC_APB1Periph_TIM2, ENABLE );
      timerHardware                   = { TIM2, GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn, 1, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1 };
      motors_gpio [ 3 ].gpio          = GPIOB;
      motors_gpio [ 3 ].pin           = Pin_1;
      motors_gpio [ 3 ].RCC_AHBPeriph = RCC_AHBPeriph_GPIOB;
#endif
      cfg.pin   = motors_gpio [ 3 ].pin;
      cfg.mode  = Mode_Out_PP;
      cfg.speed = Speed_2MHz;
      RCC_AHBPeriphClockCmd ( motors_gpio [ 3 ].RCC_AHBPeriph, ENABLE );
      gpioInit ( motors_gpio [ 3 ].gpio, &cfg );
      digitalHi ( motors_gpio [ 3 ].gpio, motors_gpio [ 3 ].pin );

      GPIO_PinAFConfig ( timerHardware.gpio, ( uint16_t ) timerHardware.gpioPinSource, timerHardware.alternateFunction );
      userMotor [ 3 ] = pwmOutConfig ( &timerHardware, PWM_BRUSHED_TIMER_MHZ, hz / masterConfig.motor_pwm_rate, 0 );
      break;

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
      *( userMotor [ 0 ]->ccr ) = ( pwmValue - 1000 ) * userMotor [ 0 ]->period / 1000;
      break;

    case M2:
      *( userMotor [ 1 ]->ccr ) = ( pwmValue - 1000 ) * userMotor [ 1 ]->period / 1000;
      break;

    case M3:
      *( userMotor [ 2 ]->ccr ) = ( pwmValue - 1000 ) * userMotor [ 2 ]->period / 1000;
      break;

    case M4:
      *( userMotor [ 3 ]->ccr ) = ( pwmValue - 1000 ) * userMotor [ 3 ]->period / 1000;
      break;

    default:
      break;
  }
}



void Motor_P::setDirection ( reverse_motor_e motor, motor_direction_e direction ) {
  // Check if the motor index is within the valid range
  if ( motor >= 0 && motor < sizeof ( motors_gpio ) / sizeof ( Rev_Motor_Gpio ) ) {
    if ( direction )
      digitalHi ( motors_gpio [ motor ].gpio, motors_gpio [ motor ].pin );
    else
      digitalLo ( motors_gpio [ motor ].gpio, motors_gpio [ motor ].pin );
  }
}



#if defined( PRIMUSX2 )
void reverseMotorGPIOInit ( void ) {

  // M2
  gpio = GPIOB;

  cfg.pin   = Pin_5;
  cfg.mode  = Mode_Out_PP;
  cfg.speed = Speed_2MHz;
  RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );
  gpioInit ( gpio, &cfg );
  digitalLo ( GPIOB, Pin_5 );

  // M3
  gpio = GPIOB;

  cfg.pin   = Pin_7;
  cfg.mode  = Mode_Out_PP;
  cfg.speed = Speed_2MHz;
  RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );
  gpioInit ( gpio, &cfg );
  digitalLo ( GPIOB, Pin_7 );

  // M4
  gpio = GPIOB;

  cfg.pin   = Pin_6;
  cfg.mode  = Mode_Out_PP;
  cfg.speed = Speed_2MHz;
  RCC_AHBPeriphClockCmd ( RCC_AHBPeriph_GPIOB, ENABLE );
  gpioInit ( gpio, &cfg );
  digitalLo ( GPIOB, Pin_6 );
}

#endif


Motor_P Motor;
