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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/maths.h"

#include "config/runtime_config.h"

#include "API/API-Utils.h"
#include "io/beeper.h"
#include "drivers/light_led.h"
#include "drivers/system.h"
#include "rx/rx.h"


uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint16_t flightIndicatorFlag = 0;
static uint32_t enabledSensors = 0;
bool rc_connected;
/**
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t enableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

/**
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t disableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}


void set_FSI(FlightStatus_e flag)
{
    flightIndicatorFlag |= (1 << flag);
}


void reset_FSI(FlightStatus_e flag)
{
    flightIndicatorFlag &= ~(1 << flag);
}


bool status_FSI(FlightStatus_e flag)
{
    return flightIndicatorFlag & (1 << flag);
}

#ifdef FLIGHT_STATUS_INDICATOR

void flightStatusIndicator(void)
{
    int32_t LedTime;
    static int delay_time=100;   //reduce this to make led blink faster
    static int32_t ActiveTime = 500;
    static uint8_t counter=0;
    static uint8_t toggle_switch=1;
    LedTime = millis();//indicates the current time in milliseconds//
    if ((int32_t)(LedTime - ActiveTime) >= delay_time && FlightStatusEnabled) { //LedTime - ActiveTime is the time for which the LED should be ON//
        counter++;
        switch (leastSignificantBit(flightIndicatorFlag)) {
            case Accel_Gyro_Calibration: {
                delay_time = 100;

                if(toggle_switch){

                ledOperator(LEDm,LED_OFF);
                ledOperator(LEDr,LED_ON);
                ledOperator(LEDl,LED_ON);

                toggle_switch=0;

                }else {

                    ledOperator(LEDm,LED_OFF);
                    ledOperator(LEDr,LED_OFF);
                    ledOperator(LEDl,LED_OFF);

                    toggle_switch=1;

                }

            }
            break;
            case Mag_Calibration: {

                delay_time = 100;

                if(toggle_switch){

                ledOperator(LEDm,LED_ON);
                ledOperator(LEDr,LED_OFF);
                ledOperator(LEDl,LED_ON);

                toggle_switch=0;

                }else {

                    ledOperator(LEDm,LED_OFF);
                    ledOperator(LEDr,LED_OFF);
                    ledOperator(LEDl,LED_OFF);

                    toggle_switch=1;

                }

            }
            break;
            case Ok_to_arm: {
                if(rc_connected) {
                    delay_time = 100;
                    ledOperator(LEDm,LED_ON);
                    ledOperator(LEDr,LED_OFF);
                    ledOperator(LEDl,LED_OFF);
                } else {
                    delay_time = 120;
                    //ledOperator(counter%3,LED_TOGGLE);
                    switch(counter%4) {
                        case 0: {
                            ledOperator(LEDl,LED_ON);
                            ledOperator(LEDm,LED_OFF);
                            ledOperator(LEDr,LED_OFF);
                            break;}

                        case 1: {
                            ledOperator(LEDl,LED_ON);
                            ledOperator(LEDm,LED_ON);
                            ledOperator(LEDr,LED_OFF);
                            break;}

                        case 2: {
                            ledOperator(LEDl,LED_ON);
                            ledOperator(LEDm,LED_ON);
                            ledOperator(LEDr,LED_ON);
                            break;}

                        case 3: {
                            ledOperator(LEDl,LED_OFF);
                            ledOperator(LEDm,LED_OFF);
                            ledOperator(LEDr,LED_OFF);
                            break;}

                    }
                }
            }
            break;
            case Not_ok_to_arm: {
                if(rc_connected) {
                delay_time = 100;
                ledOperator(LEDm,LED_TOGGLE);
                ledOperator(LEDr,LED_OFF);
                ledOperator(LEDl,LED_OFF);
                }
                else {
                        delay_time = 120;
                        //ledOperator(counter%3,LED_TOGGLE);
                        switch(counter%4) {
                            case 0: {
                                ledOperator(LEDl,LED_ON);
                                ledOperator(LEDm,LED_OFF);
                                ledOperator(LEDr,LED_OFF);
                                break;}

                            case 1: {
                                ledOperator(LEDl,LED_ON);
                                ledOperator(LEDm,LED_ON);
                                ledOperator(LEDr,LED_OFF);
                                break;}

                            case 2: {
                                ledOperator(LEDl,LED_ON);
                                ledOperator(LEDm,LED_ON);
                                ledOperator(LEDr,LED_ON);
                                break;}

                            case 3: {
                                ledOperator(LEDl,LED_OFF);
                                ledOperator(LEDm,LED_OFF);
                                ledOperator(LEDr,LED_OFF);
                                break;}
                        }
                   }
            }
            break;
            case Armed: {
                if(rc_connected) {
                    delay_time = 100;

                    ledOperator(LEDm,LED_OFF);
                    ledOperator(LEDr,LED_ON);
                    ledOperator(LEDl,LED_OFF);
                }
            }
            break;
            case LowBattery_inFlight: {                   //to indicate that battery is too low to arm//
                delay_time = 100;
                ledOperator(LEDm,LED_OFF);
                ledOperator(LEDr,LED_OFF);
                ledOperator(LEDl,LED_TOGGLE);
            }
            break;
            case Low_battery: {                 //to indicate that battery is to low during flight//
                delay_time = 100;
                ledOperator(LEDm,LED_OFF);
                ledOperator(LEDr,LED_OFF);
                ledOperator(LEDl,LED_ON);
            }
            break;
            case Signal_loss: {                  //to indicate that signal loss has occurred//
                delay_time = 100;
                ledOperator(LEDm,LED_OFF);
                ledOperator(LEDr,LED_TOGGLE);
                ledOperator(LEDl,LED_OFF);

            }
            break;
            case Crash: {
                delay_time = 100;
                switch(counter%2) {

                    case 0: {
                        ledOperator(LEDl,LED_ON);
                        ledOperator(LEDm,LED_OFF);
                        ledOperator(LEDr,LED_OFF);
                        break;}

                    case 1: {
                        ledOperator(LEDl,LED_OFF);
                        ledOperator(LEDm,LED_OFF);
                        ledOperator(LEDr,LED_ON);
                        break;}
                }
            }
            break;
        }
        ActiveTime = LedTime + delay_time;

    }
}
#endif
