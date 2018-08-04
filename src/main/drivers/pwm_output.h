/*
 * This file is part of Cleanflight and Magis.
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
#pragma once

#ifdef __cplusplus
extern "C" {
#endif 

#include <stdbool.h>
#include <stdint.h>

#include <stdlib.h>

#include "platform.h"

#include "gpio.h"
#include "timer.h"

#include "flight/failsafe.h" // FIXME dependency into the main code from a driver

#include "pwm_mapping.h"

#include "pwm_output.h"

typedef void (*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors




typedef struct {
        volatile timCCR_t *ccr;
        TIM_TypeDef *tim;
        uint16_t period;
        pwmWriteFuncPtr pwmWritePtr;
} pwmOutputPort_t;


 pwmOutputPort_t *pwmOutConfig(const timerHardware_t *timerHardware, uint8_t mhz, uint16_t period, uint16_t value);

void pwmWriteMotor(uint8_t index, uint16_t value);
void pwmShutdownPulsesForAllMotors(uint8_t motorCount);
void pwmCompleteOneshotMotorUpdate(uint8_t motorCount);

void pwmWriteServo(uint8_t index, uint16_t value);

bool isMotorBrushed(uint16_t motorPwmRate);

#ifdef __cplusplus
}
#endif 
