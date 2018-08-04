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

void systemInit(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint32_t micros(void);
uint32_t millis(void);

#ifdef TEST_ENABLE
// failure
void failureMode(uint8_t mode);
#endif

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(void);
bool isMPUSoftReset(void);

void enableGPIOPowerUsageAndNoiseReductions(void);
// current crystal frequency - 8 or 12MHz
extern uint32_t hse_value;

typedef void extiCallbackHandlerFunc(void);

void registerExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);
void unregisterExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn);

extern uint32_t cachedRccCsrValue;

typedef enum {                 //DD
    FAILURE_DEVELOPER = 0,
    FAILURE_MISSING_ACC,
    //FAILURE_ACC_INIT,
    FAILURE_ACC_INCOMPATIBLE,
    FAILURE_INVALID_EEPROM_CONTENTS,
    FAILURE_FLASH_WRITE_FAILED,
    //FAILURE_GYRO_INIT_FAILED
    FAILURE_BARO,
    FAILURE_EXTCLCK
} failureMode_e;
extern uint8_t failureFlag;

#ifdef __cplusplus
}
#endif 
