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
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/ranging_vl53l0x.h"
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
#include "io/flashfs.h"
#include "io/oled_display.h"

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

#include "mw.h"

#include "API/API-Utils.h"
#include "API/Utils.h"
#include "API/User.h"
#include "API/Estimate.h"
#include "API/Specifiers.h"
#include "API/Peripheral.h"
#include "API/Motor.h"
#include "API/PlutoPilot.h"
#include "API/XRanging.h"
#include "API/Localisation.h"
#include "API/RxConfig.h"
#include "command/command.h"
#include "command/localisationCommand.h"
#include "drivers/opticflow_paw3903.h"

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL ( 6 * 3500 )
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL ( 6 * 3500 )

int8_t returnValue         = 100;
uint8_t motorControlEnable = false;
extern uint8_t dynP8 [ 3 ];
extern uint8_t dynI8 [ 3 ];
extern uint8_t dynD8 [ 3 ];
extern uint8_t PIDweight [ 3 ];

int16_t magHold;
int16_t headFreeModeHold = 83;
int16_t telemTemperature1;      // gyro sensor temperature
uint16_t cycleTime      = 0;    // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
int16_t stateHandler    = 0;
int16_t startEstimation = 0;

uint32_t currentTime  = 0;
uint32_t previousTime = 0;
uint32_t timeNow;
uint32_t timeDifference;
uint32_t add1           = 23;
uint32_t debugThrottle  = 10;
uint32_t debugThrottle1 = 20;
uint32_t current_time   = 0;
uint32_t total_time     = 0;
uint32_t previous_time  = 0;
uint32_t mode_checker   = 0;
static uint32_t disarmAt;    // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

uint32_t userCurrentTime = 0;
uint32_t userLoopTime    = 0;

bool receivingRxData;
bool isHeadSet          = false;
bool isCalibrated       = false;
bool shouldStart        = false;
bool tookOff            = false;
bool crashRecoveryCheck = false;
static bool isRXDataNew;
static bool isBatteryLow = false;
uint32_t arm_time        = 0;

bool isBlackboxOn = false;

// bool isFlipDone=false;
// bool startStable=false;

enum {
  ALIGN_GYRO  = 0,
  ALIGN_ACCEL = 1,
  ALIGN_MAG   = 2
};

// Interval crashTimer;

Interval dataLog;    // Timer for controlling logging interval

int16_t myRC [ 8 ] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

typedef void ( *pidControllerFuncPtr ) ( pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig );    // pid controller function prototype

extern pidControllerFuncPtr pid_controller;

uint8_t getPos [ 2 ] = { 0x02, 0x00 };
uint8_t ch;

Interval UartTimer;
uint32_t loopTime3;

uint32_t Uwbtime;

// UWB Data Read

#ifdef UWB
void uwbUpdate ( ) {

  if ( GPIO.read ( Pin8 ) ) {    // When UWB TAG gets new data GPIO 8 on UNIBUS gets HIGH: works as a ready flag

    LED_M_TOGGLE;                // Toggling green LED for data received indication

    UART.write ( UART2, getPos, 2 );

    while ( UART.rxBytesWaiting ( UART2 ) ) {
      ch = UART.read8 ( UART2 );
      if ( ch == 0x40 ) {
        ch = UART.read8 ( UART2 );
        if ( ch == 0x01 ) {
          ch = UART.read8 ( UART2 );
          if ( ch == 0x00 ) {
            ch = UART.read8 ( UART2 );
            if ( ch == 0x41 ) {
              ch      = UART.read8 ( UART2 );
              posX    = ( int16_t ) ( ( ( int32_t ) UART.read32 ( UART2 ) ) / 10 );
              posY    = ( int16_t ) ( ( ( int32_t ) UART.read32 ( UART2 ) ) / 10 );
              posZ    = ( int16_t ) ( ( ( int32_t ) UART.read32 ( UART2 ) ) / 10 );
              Quality = ( int8_t ) ( ( int32_t ) UART.read8 ( UART2 ) );

              if ( Quality > 51 )    // only if quality is greater than 50% take as a new position
                new_position = true;
            }
          }
        }
      }
    }
  }
}

#endif

void applyAndSaveAccelerometerTrimsDelta ( rollAndPitchTrims_t *rollAndPitchTrimsDelta ) {
  currentProfile->accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
  currentProfile->accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;
  saveConfigAndNotify ( );
}

#ifdef GTUNE

void updateGtuneState ( void ) {
  static bool GTuneWasUsed = false;

  if ( IS_RC_MODE_ACTIVE ( BOXGTUNE ) ) {
    if ( ! FLIGHT_MODE ( GTUNE_MODE ) && ARMING_FLAG ( ARMED ) ) {
      ENABLE_FLIGHT_MODE ( GTUNE_MODE );
      init_Gtune ( &currentProfile->pidProfile );
      GTuneWasUsed = true;
    }
    if ( ! FLIGHT_MODE ( GTUNE_MODE ) && ! ARMING_FLAG ( ARMED ) && GTuneWasUsed ) {
      saveConfigAndNotify ( );
      GTuneWasUsed = false;
    }
  } else {
    if ( FLIGHT_MODE ( GTUNE_MODE ) && ARMING_FLAG ( ARMED ) ) {
      DISABLE_FLIGHT_MODE ( GTUNE_MODE );
    }
  }
}
#endif

bool isCalibrating ( ) {

  /*
  #ifdef BARO
      if (sensors(SENSOR_BARO) && !isBaroCalibrationComplete()) {
          //return true;
      }
  #endif
  */
  // Note: compass calibration is handled completely differently, outside of the main loop, see f.CALIBRATE_MAG
#ifdef MAG_ENFORCE
  // to check whether mag calibration is done, if not the don't arm
  if ( masterConfig.magScale.raw [ 0 ] == 0 || masterConfig.magScale.raw [ 1 ] == 0 || masterConfig.magScale.raw [ 2 ] == 0 ) {
    set_FSI ( Mag_Calibration );
  } else {
    reset_FSI ( Mag_Calibration );
  }
#endif
  if ( isAccelerationCalibrationComplete ( ) && isGyroCalibrationComplete ( ) )
    reset_FSI ( Accel_Gyro_Calibration );
  else
    set_FSI ( Accel_Gyro_Calibration );

  return ( status_FSI ( Accel_Gyro_Calibration ) || accZoffsetCalCycle
#ifdef MAG_ENFORCE
           || status_FSI ( Mag_Calibration )
#endif
  );
}

void annexCode ( void ) {
  int32_t tmp;
  int32_t tmp2;
  int32_t axis;
  int32_t prop1 = 0;
  int32_t prop2;

  static uint32_t vbatLastServiced = 0;
  static uint32_t ibatLastServiced = 0;

  if ( rcData [ THROTTLE ] < currentControlRateProfile->tpa_breakpoint ) {
    prop2 = 100;
  } else {
    if ( rcData [ THROTTLE ] < 2000 ) {
      prop2 = 100 - ( uint16_t ) currentControlRateProfile->dynThrPID * ( rcData [ THROTTLE ] - currentControlRateProfile->tpa_breakpoint ) / ( 2000 - currentControlRateProfile->tpa_breakpoint );
    } else {
      prop2 = 100 - currentControlRateProfile->dynThrPID;
    }
  }

  for ( axis = 0; axis < 3; axis++ ) {
    tmp = MIN ( ABS ( rcData [ axis ] - masterConfig.rxConfig.midrc ), 500 );
    if ( axis == ROLL || axis == PITCH ) {
      if ( currentProfile->rcControlsConfig.deadband ) {
        if ( tmp > currentProfile->rcControlsConfig.deadband ) {
          tmp -= currentProfile->rcControlsConfig.deadband;
        } else {
          tmp = 0;
        }
      }

      tmp2               = tmp / 100;
      rcCommand [ axis ] = lookupPitchRollRC [ tmp2 ] + ( tmp - tmp2 * 100 ) * ( lookupPitchRollRC [ tmp2 + 1 ] - lookupPitchRollRC [ tmp2 ] ) / 100;
      prop1              = 100 - ( uint16_t ) currentControlRateProfile->rates [ axis ] * tmp / 500;
      prop1              = ( uint16_t ) prop1 * prop2 / 100;

    } else if ( axis == YAW ) {
      if ( currentProfile->rcControlsConfig.yaw_deadband ) {
        if ( tmp > currentProfile->rcControlsConfig.yaw_deadband ) {
          tmp -= currentProfile->rcControlsConfig.yaw_deadband;
        } else {
          tmp = 0;
        }
      }
      tmp2               = tmp / 100;
      rcCommand [ axis ] = ( lookupYawRC [ tmp2 ] + ( tmp - tmp2 * 100 ) * ( lookupYawRC [ tmp2 + 1 ] - lookupYawRC [ tmp2 ] ) / 100 ) * -masterConfig.yaw_control_direction;
      prop1              = 100 - ( uint16_t ) currentControlRateProfile->rates [ axis ] * ABS ( tmp ) / 500;
    }
    // FIXME axis indexes into pids.  use something like lookupPidIndex(rc_alias_e alias) to reduce coupling.
    dynP8 [ axis ] = ( uint16_t ) currentProfile->pidProfile.P8 [ axis ] * prop1 / 100;
    dynI8 [ axis ] = ( uint16_t ) currentProfile->pidProfile.I8 [ axis ] * prop1 / 100;
    dynD8 [ axis ] = ( uint16_t ) currentProfile->pidProfile.D8 [ axis ] * prop1 / 100;

    // non coupled PID reduction scaler used in PID controller 1 and PID controller 2. YAW TPA disabled. 100 means 100% of the pids
    if ( axis == YAW ) {
      PIDweight [ axis ] = 100;
    } else {
      PIDweight [ axis ] = prop2;
    }

    if ( rcData [ axis ] < masterConfig.rxConfig.midrc )
      rcCommand [ axis ] = -rcCommand [ axis ];
  }

  if ( limitAltitude ( ) && rcData [ THROTTLE ] > 1500 ) {
    rcData [ THROTTLE ] = 1500;
  }

  if ( isLanding ) {
    rcData [ THROTTLE ] = landThrottle;
  }

  tmp                    = constrain ( rcData [ THROTTLE ], masterConfig.rxConfig.mincheck, PWM_RANGE_MAX );
  tmp                    = ( uint32_t ) ( tmp - masterConfig.rxConfig.mincheck ) * PWM_RANGE_MIN / ( PWM_RANGE_MAX - masterConfig.rxConfig.mincheck );    // [MINCHECK;2000] -> [0;1000]
  tmp2                   = tmp / 100;
  rcCommand [ THROTTLE ] = lookupThrottleRC [ tmp2 ] + ( tmp - tmp2 * 100 ) * ( lookupThrottleRC [ tmp2 + 1 ] - lookupThrottleRC [ tmp2 ] ) / 100;        // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

  if ( isLocalisationOn && ARMING_FLAG ( ARMED ) ) {                                                                                                      // for localisation
    rcCommand [ ROLL ]  = getrcDataRoll ( );
    rcCommand [ PITCH ] = getrcDataPitch ( );

  } else {

    resetPosIntegral ( );
    command_jump ( -1 );
  }

  if ( FLIGHT_MODE ( HEADFREE_MODE ) ) {
    float radDiff           = degreesToRadians ( heading - headFreeModeHold );
    float cosDiff           = cos_approx ( radDiff );
    float sinDiff           = sin_approx ( radDiff );
    int16_t rcCommand_PITCH = rcCommand [ PITCH ] * cosDiff + rcCommand [ ROLL ] * sinDiff;
    rcCommand [ ROLL ]      = rcCommand [ ROLL ] * cosDiff - rcCommand [ PITCH ] * sinDiff;
    rcCommand [ PITCH ]     = rcCommand_PITCH;
  }

  if ( feature ( FEATURE_VBAT ) ) {
    if ( cmp32 ( currentTime, vbatLastServiced ) >= VBATINTERVAL ) {
      vbatLastServiced = currentTime;
      updateBattery ( );
    }
  }

  if ( feature ( FEATURE_CURRENT_METER ) ) {
    int32_t ibatTimeSinceLastServiced = cmp32 ( currentTime, ibatLastServiced );

    if ( ibatTimeSinceLastServiced >= IBATINTERVAL ) {
      ibatLastServiced = currentTime;
      updateCurrentMeter ( ibatTimeSinceLastServiced, &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle );
    }
  }

  beeperUpdate ( );    // call periodic beeper handler

  if ( ARMING_FLAG ( ARMED ) ) {
    set_FSI ( Armed );
    reset_FSI ( Ok_to_arm );
    reset_FSI ( Not_ok_to_arm );
  } else {
    reset_FSI ( Armed );
    if ( IS_RC_MODE_ACTIVE ( BOXARM ) == 0 ) {

      // LED_L_ON;
      if ( ! STATE ( SMALL_ANGLE ) || ! isCalibrated ) {
        DISABLE_ARMING_FLAG ( OK_TO_ARM );
        reset_FSI ( Ok_to_arm );
        set_FSI ( Not_ok_to_arm );
      } else {

        ENABLE_ARMING_FLAG ( OK_TO_ARM );
        set_FSI ( Ok_to_arm );
        reset_FSI ( Not_ok_to_arm );
      }
    }

    if ( status_FSI ( Crash ) && ! crashRecoveryCheck ) {

      if ( crashTimer.set ( 300, false ) ) {

        crashRecoveryCheck = true;
        crashTimer.reset ( );
      }
    }

    if ( crashRecoveryCheck && STATE ( SMALL_ANGLE ) ) {
      reset_FSI ( Crash );
      crashRecoveryCheck = false;
    }

    if ( vbat > 34 ) {
      isBatteryLow = false;
      // ENABLE_ARMING_FLAG(OK_TO_ARM);???
      reset_FSI ( Low_battery );
    } else if ( fsLowBattery ) {
      isBatteryLow = true;
      set_FSI ( Low_battery );
    }
    //   }

    if ( isCalibrating ( ) ) {
      warningLedFlash ( );
      DISABLE_ARMING_FLAG ( OK_TO_ARM );
      // flightIndicatorFlag|=(1<<Accel_Gyro_Calibration);
    } else {
      // flightIndicatorFlag&=~(1<<Accel_Gyro_Calibration);
      if ( ARMING_FLAG ( OK_TO_ARM ) ) {
        warningLedDisable ( );
      } else {
        warningLedFlash ( );
      }
    }
#ifdef FLIGHT_STATUS_INDICATOR
    // warningLedUpdate();
#endif
  }

#ifdef TELEMETRY
  telemetryCheckState ( );
#endif

  handleSerial ( );

#ifdef GPS
  if ( sensors ( SENSOR_GPS ) ) {
    updateGpsIndicator ( currentTime );
  }
#endif

  // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
  if ( gyro.temperature )
    gyro.temperature ( &telemTemperature1 );
}

void mwDisarm ( void ) {

  if ( ARMING_FLAG ( ARMED ) ) {
    DISABLE_ARMING_FLAG ( ARMED );
    DISABLE_ARMING_FLAG ( OK_TO_ARM );

#ifdef BLACKBOX
    if ( feature ( FEATURE_BLACKBOX ) ) {
      finishBlackbox ( );
    }
#endif

    beeper ( BEEPER_DISARMING );    // emit disarm tone
  }

  if ( current_command != NONE )
    command_status = FINISHED;
  isLanding          = false;
  setLandTimer       = true;
  setTakeOffAlt      = false;
  setTakeOffThrottle = false;
  setTakeOffTimer    = true;
  isTookOff          = false;
  isTakeOffHeightSet = false;
  takeOffThrottle    = 950;
  takeOffHeight      = 120;
  landThrottle       = 1200;
  flipState          = 0;
  arm_time           = 0;
  reset_FSI ( LowBattery_inFlight );
  reset_FSI ( Signal_loss );

  isPitchStabelised    = false;
  isRollStabelised     = false;
  isHeadSet            = false;
  isChukedArmed        = false;
  isThrottleStickArmed = false;
  // Control.setFailsafeState(CRASH, true);

  resetPosController ( );
}

#define TELEMETRY_FUNCTION_MASK ( FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_MSP | FUNCTION_TELEMETRY_SMARTPORT )

void releaseSharedTelemetryPorts ( void ) {
  serialPort_t *sharedPort = findSharedSerialPort ( TELEMETRY_FUNCTION_MASK, FUNCTION_MSP );
  while ( sharedPort ) {
    mspReleasePortIfAllocated ( sharedPort );
    sharedPort = findNextSharedSerialPort ( TELEMETRY_FUNCTION_MASK, FUNCTION_MSP );
  }
}

void mwArm ( void ) {

  if ( ( ARMING_FLAG ( OK_TO_ARM ) || netAccMagnitude < 2 ) && isCalibrated && ! isBatteryLow ) {
    if ( ARMING_FLAG ( ARMED ) ) {
      return;
    }
    if ( IS_RC_MODE_ACTIVE ( BOXFAILSAFE ) ) {
      return;
    }
    if ( ! ARMING_FLAG ( PREVENT_ARMING ) ) {
      ENABLE_ARMING_FLAG ( ARMED );
      if ( isUserHeadFreeHoldSet ) {

        headFreeModeHold = userHeadFreeHoldHeading;

      } else
        headFreeModeHold = heading;

      // headFreeModeHold=83;

      arm_time = millis ( );

#ifdef BLACKBOX
      if ( feature ( FEATURE_BLACKBOX ) ) {
        serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort ( FUNCTION_BLACKBOX, FUNCTION_MSP );
        if ( sharedBlackboxAndMspPort ) {
          mspReleasePortIfAllocated ( sharedBlackboxAndMspPort );
        }
        startBlackbox ( );
      }
#endif
      disarmAt = millis ( ) + masterConfig.auto_disarm_delay * 1000;    // start disarm timeout, will be extended when throttle is nonzero

	                      // beep to indicate arming
#ifdef GPS
      if ( feature ( FEATURE_GPS ) && STATE ( GPS_FIX ) && GPS_numSat >= 5 )
        beeper ( BEEPER_ARMING_GPS_FIX );
      else
        beeper ( BEEPER_ARMING );
#else
      beeper ( BEEPER_ARMING );
#endif

      return;
    }
  }

  if ( ! ARMING_FLAG ( ARMED ) ) {
    beeperConfirmationBeeps ( 1 );
  }
}

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed           = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm    = false;
bool AccInflightCalibrationActive          = false;
uint16_t InflightcalibratingA              = 0;

void handleInflightCalibrationStickPosition ( void ) {
  if ( AccInflightCalibrationMeasurementDone ) {
    // trigger saving into eeprom after landing
    AccInflightCalibrationMeasurementDone = false;
    AccInflightCalibrationSavetoEEProm    = true;
  } else {
    AccInflightCalibrationArmed = ! AccInflightCalibrationArmed;
    if ( AccInflightCalibrationArmed ) {
      beeper ( BEEPER_ACC_CALIBRATION );
    } else {
      beeper ( BEEPER_ACC_CALIBRATION_FAIL );
    }
  }
}

void updateInflightCalibrationState ( void ) {
  if ( AccInflightCalibrationArmed && ARMING_FLAG ( ARMED ) && rcData [ THROTTLE ] > masterConfig.rxConfig.mincheck && ! IS_RC_MODE_ACTIVE ( BOXARM ) ) {    // Copter is airborne and you are turning it off via boxarm : start measurement
    InflightcalibratingA        = 50;
    AccInflightCalibrationArmed = false;
  }
  if ( IS_RC_MODE_ACTIVE ( BOXCALIB ) ) {    // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
    if ( ! AccInflightCalibrationActive && ! AccInflightCalibrationMeasurementDone )
      InflightcalibratingA = 50;
    AccInflightCalibrationActive = true;
  } else if ( AccInflightCalibrationMeasurementDone && ! ARMING_FLAG ( ARMED ) ) {
    AccInflightCalibrationMeasurementDone = false;
    AccInflightCalibrationSavetoEEProm    = true;
  }
}

void updateMagHold ( void ) {
  static bool isMagHoldChanged = false;
  int16_t headingError;

  if ( ABS ( rcData [ YAW ] - 1500 ) < 300 && ( FLIGHT_MODE ( MAG_MODE ) || FLIGHT_MODE ( HEADFREE_MODE ) ) ) {

    if ( isMagHoldChanged && ABS ( gyroADC [ YAW ] / 4 ) < 70 ) {
      isMagHoldChanged = false;
      magHold          = heading;
    }

    if ( ! isMagHoldChanged ) {
      headingError = heading - magHold;
      if ( headingError <= -180 )
        headingError += 360;
      if ( headingError >= +180 )
        headingError -= 360;

      headingError = constrain ( headingError, -30, 30 );

      rcCommand [ YAW ] = headingError * currentProfile->pidProfile.P8 [ PIDMAG ];
      rcCommand [ YAW ] = constrain ( rcCommand [ YAW ], -400, 400 );
    }

  } else {

    isMagHoldChanged = true;
    magHold          = heading;
  }
}

typedef enum {
#ifdef MAG
  UPDATE_COMPASS_TASK,
#endif
#ifdef BARO
  UPDATE_BARO_TASK,
#endif
#ifdef SONAR
  UPDATE_SONAR_TASK,
#endif
#if defined( BARO ) || defined( SONAR )
  CALCULATE_ALTITUDE_TASK,
#endif
#ifdef DISPLAY
  UPDATE_DISPLAY_TASK,
#endif

  UPDATE_OPTICFLOW,

  UPDATE_LASER_TOF_TASK,

} periodicTasks;

#define PERIODIC_TASK_COUNT ( UPDATE_LASER_TOF_TASK + 1 )

void executePeriodicTasks ( void ) {

  static int periodicTaskIndex = 0;

  switch ( periodicTaskIndex++ ) {
#ifdef MAG
    case UPDATE_COMPASS_TASK:
      if ( sensors ( SENSOR_MAG ) ) {
        updateCompass ( &masterConfig.magZero, &masterConfig.magScale );
      }
      break;
#endif

#ifdef BARO
    case UPDATE_BARO_TASK:
      if ( sensors ( SENSOR_BARO ) ) {

        baroUpdate ( currentTime );
      }
      break;
#endif

#if defined( BARO ) || defined( SONAR )
    case CALCULATE_ALTITUDE_TASK:
      if ( true
  #if defined( BARO )
           || ( sensors ( SENSOR_BARO ) && isBaroReady ( ) )
  #endif
  #if defined( SONAR )
           || sensors ( SENSOR_SONAR )
  #endif
      ) {

        if ( ! isCalibrating ( ) ) {

          apmCalculateEstimatedAltitude ( currentTime );

          if ( localisationType == UWB )
            PosXYEstimate ( currentTime );

          isCalibrated = true;

        }

        else {

          isCalibrated = false;
        }
      }
      break;
#endif
#ifdef SONAR
    case UPDATE_SONAR_TASK:
      if ( sensors ( SENSOR_SONAR ) ) {
        sonarUpdate ( );
      }
      break;
#endif
#ifdef DISPLAY
    case UPDATE_DISPLAY_TASK:
      if ( feature ( FEATURE_DISPLAY ) ) {
        updateDisplay ( );
      }
      break;
#endif

    case UPDATE_OPTICFLOW:

#ifdef OPTIC_FLOW
      updateSpiOpticFlow ( );
      runFlowHold ( currentTime );
#endif
      break;

    case UPDATE_LASER_TOF_TASK:

#ifdef LASER_TOF

      getRange ( );

#endif
      //        if(useRangingSensor)
      //            getRange();


      break;
  }

  if ( periodicTaskIndex >= PERIODIC_TASK_COUNT ) {
    periodicTaskIndex = 0;
  }
}

void processRx ( void ) {

  static bool armedBeeperOn = false;

  calculateRxChannelsAndUpdateFailsafe ( currentTime );

  // in 3D mode, we need to be able to disarm by switch at any time

  if ( feature ( FEATURE_3D ) ) {
    if ( ! IS_RC_MODE_ACTIVE ( BOXARM ) )
      mwDisarm ( );
  }

  updateRSSI ( currentTime );

  if ( feature ( FEATURE_FAILSAFE ) ) {

    if ( currentTime > FAILSAFE_POWER_ON_DELAY_US && ! failsafeIsMonitoring ( ) ) {
      failsafeStartMonitoring ( );    // nothing much; monitering = true
    }

    failsafeUpdateState ( );
  }

  throttleStatus_e throttleStatus = calculateThrottleStatus ( &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle );

  if ( throttleStatus == THROTTLE_LOW ) {
    pidResetErrorAngle ( );
    pidResetErrorGyro ( );
  }

  // When armed and motors aren't spinning, do beeps and then disarm
  // board after delay so users without buzzer won't lose fingers.
  // mixTable constrains motor commands, so checking  throttleStatus is enough
  if ( ARMING_FLAG ( ARMED ) && feature ( FEATURE_MOTOR_STOP ) && ! STATE ( FIXED_WING ) ) {
    if ( isUsingSticksForArming ( ) ) {
      if ( throttleStatus == THROTTLE_LOW ) {
        if ( masterConfig.auto_disarm_delay != 0 && ( int32_t ) ( disarmAt - millis ( ) ) < 0 ) {
          // auto-disarm configured and delay is over
          mwDisarm ( );
          armedBeeperOn = false;
        } else {
          // still armed; do warning beeps while armed
          beeper ( BEEPER_ARMED );
          armedBeeperOn = true;
        }
      } else {
        // throttle is not low
        if ( masterConfig.auto_disarm_delay != 0 ) {
          // extend disarm time
          disarmAt = millis ( ) + masterConfig.auto_disarm_delay * 1000;
        }

        if ( armedBeeperOn ) {
          beeperSilence ( );
          armedBeeperOn = false;
        }
      }
    } else {
      // arming is via AUX switch; beep while throttle low
      if ( throttleStatus == THROTTLE_LOW ) {
        beeper ( BEEPER_ARMED );
        armedBeeperOn = true;
      } else if ( armedBeeperOn ) {
        beeperSilence ( );
        armedBeeperOn = false;
      }
    }
  }

  processRcStickPositions ( &masterConfig.rxConfig, throttleStatus, masterConfig.retarded_arm, masterConfig.disarm_kill_switch );

  if ( feature ( FEATURE_INFLIGHT_ACC_CAL ) ) {
    updateInflightCalibrationState ( );
  }

  updateActivatedModes ( currentProfile->modeActivationConditions );

  if ( ! cliMode ) {
    updateAdjustmentStates ( currentProfile->adjustmentRanges );
    processRcAdjustments ( currentControlRateProfile, &masterConfig.rxConfig );
  }

  bool canUseHorizonMode = true;

  if ( ( IS_RC_MODE_ACTIVE ( BOXANGLE ) || ( feature ( FEATURE_FAILSAFE ) && failsafeIsActive ( ) ) ) && ( sensors ( SENSOR_ACC ) ) ) {
    // bumpless transfer to Level mode
    canUseHorizonMode = false;

    if ( ! FLIGHT_MODE ( ANGLE_MODE ) ) {
      pidResetErrorAngle ( );
      ENABLE_FLIGHT_MODE ( ANGLE_MODE );
    }
  } else {
    DISABLE_FLIGHT_MODE ( ANGLE_MODE );    // failsafe support
  }

  if ( IS_RC_MODE_ACTIVE ( BOXHORIZON ) && canUseHorizonMode ) {

    DISABLE_FLIGHT_MODE ( ANGLE_MODE );

    if ( ! FLIGHT_MODE ( HORIZON_MODE ) ) {
      pidResetErrorAngle ( );
      ENABLE_FLIGHT_MODE ( HORIZON_MODE );
    }
  } else {
    DISABLE_FLIGHT_MODE ( HORIZON_MODE );
  }

#ifdef MAG
  if ( ! ARMING_FLAG ( ARMED ) && ( FLIGHT_MODE ( MAG_MODE ) || FLIGHT_MODE ( HEADFREE_MODE ) ) ) {
    magHold = heading;    // initialization
  }
  if ( sensors ( SENSOR_ACC ) || sensors ( SENSOR_MAG ) ) {
    if ( IS_RC_MODE_ACTIVE ( BOXMAG ) ) {
      if ( ! FLIGHT_MODE ( MAG_MODE ) ) {
        ENABLE_FLIGHT_MODE ( MAG_MODE );
        magHold = heading;    // in flight initialization
      }
    } else {
      DISABLE_FLIGHT_MODE ( MAG_MODE );
    }
    if ( IS_RC_MODE_ACTIVE ( BOXHEADFREE ) ) {
      if ( ! FLIGHT_MODE ( HEADFREE_MODE ) ) {
        ENABLE_FLIGHT_MODE ( HEADFREE_MODE );
      }
    } else {
      DISABLE_FLIGHT_MODE ( HEADFREE_MODE );
    }
    if ( IS_RC_MODE_ACTIVE ( BOXHEADADJ ) ) {
      headFreeModeHold = heading;    // acquire new heading
    }

    // Used while inFligght Mode change
    if ( IS_RC_MODE_ACTIVE ( BOXBARO ) && ! isHeadSet ) {
      headFreeModeHold = heading;    // acquire new heading
      isHeadSet        = true;
    }

    if ( ! IS_RC_MODE_ACTIVE ( BOXBARO ) ) {

      isHeadSet = false;
    }
  }
#endif

#ifdef GPS
  if ( sensors ( SENSOR_GPS ) ) {

    updateGpsWaypointsAndMode ( );
  }
#endif

  if ( IS_RC_MODE_ACTIVE ( BOXPASSTHRU ) ) {
    ENABLE_FLIGHT_MODE ( PASSTHRU_MODE );
  } else {
    DISABLE_FLIGHT_MODE ( PASSTHRU_MODE );
  }

  if ( masterConfig.mixerMode == MIXER_FLYING_WING || masterConfig.mixerMode == MIXER_AIRPLANE ) {
    DISABLE_FLIGHT_MODE ( HEADFREE_MODE );
  }

#ifdef TELEMETRY
  if ( feature ( FEATURE_TELEMETRY ) ) {
    if ( ( ! masterConfig.telemetryConfig.telemetry_switch && ARMING_FLAG ( ARMED ) ) || ( masterConfig.telemetryConfig.telemetry_switch && IS_RC_MODE_ACTIVE ( BOXTELEMETRY ) ) ) {

      releaseSharedTelemetryPorts ( );

    } else {
      // the telemetry state must be checked immediately so that shared serial ports are released.
      telemetryCheckState ( );
      mspAllocateSerialPorts ( &masterConfig.serialConfig );
    }
  }
#endif
}

void filterRc ( void ) {
  static int16_t lastCommand [ 4 ] = { 0, 0, 0, 0 };
  static int16_t deltaRC [ 4 ]     = { 0, 0, 0, 0 };
  static int16_t factor, rcInterpolationFactor;
  static filterStatePt1_t filteredCycleTimeState;
  uint16_t rxRefreshRate, filteredCycleTime;

  // Set RC refresh rate for sampling and channels to filter
  initRxRefreshRate ( &rxRefreshRate );

  filteredCycleTime     = filterApplyPt1 ( cycleTime, &filteredCycleTimeState, 1 );
  rcInterpolationFactor = rxRefreshRate / filteredCycleTime + 1;

  if ( isRXDataNew ) {
    for ( int channel = 0; channel < 4; channel++ ) {
      deltaRC [ channel ]     = rcCommand [ channel ] - ( lastCommand [ channel ] - deltaRC [ channel ] * factor / rcInterpolationFactor );
      lastCommand [ channel ] = rcCommand [ channel ];
    }

    isRXDataNew = false;
    factor      = rcInterpolationFactor - 1;
  } else {
    factor--;
  }

  // Interpolate steps of rcCommand
  if ( factor > 0 ) {
    for ( int channel = 0; channel < 4; channel++ ) {
      rcCommand [ channel ] = lastCommand [ channel ] - deltaRC [ channel ] * factor / rcInterpolationFactor;
    }
  } else {
    factor = 0;
  }
}

void userCode ( ) {

  if ( ( rcData [ DevModeAUX ] >= DevModeMinRange && rcData [ DevModeAUX ] <= DevModeMaxRange ) && ( rxIsReceivingSignal ( ) || ppmIsRecievingSignal ( ) ) ) {
    runUserCode = true;
    devmode     = true;

  } else {
    runUserCode = false;
    devmode     = false;
  }

  OledDisplayData ( );

  if ( runUserCode ) {
    userCurrentTime = micros ( );
    if ( ( int32_t ) ( userCurrentTime - userLoopTime ) >= 0 ) {
      {
        userLoopTime = userCurrentTime + userLoopFrequency;    // 100ms default
        if ( callOnPilotStart ) {
          onLoopStart ( );
          callOnPilotStart  = false;
          callonPilotFinish = true;
        }

        plutoLoop ( );
      }
    } else {

      for ( int i = 0; i < 4; i++ ) {

        if ( userRCflag [ i ] ) {

          if ( i < 3 )
            rcCommand [ i ] = RC_ARRAY [ i ];
          else if ( i == 3 )
            rcData [ i ] = RC_ARRAY [ i ];
        }
      }

      if ( isUserHeadingSet ) {

        magHold = userHeading;
      }

      for ( int i = 0; i < 6; i++ ) {

        if ( isUserFlightModeSet [ i ] )
          FlightMode.set ( ( flight_mode_e ) i );
      }
    }

  }

  else {
    if ( callonPilotFinish ) {
      for ( int i = 0; i < 4; i++ ) {
        RC_ARRAY [ i ]   = 0;
        userRCflag [ i ] = false;
      }

      isUserHeadingSet = false;
      onLoopFinish ( );
      callOnPilotStart  = true;
      callonPilotFinish = false;
    }
  }
}

void loop ( void ) {

  static uint32_t loopTime;
#if defined( BARO ) || defined( SONAR )
  static bool haveProcessedAnnexCodeOnce = false;
#endif

  OledStartUpPage ( );

  updateRx ( currentTime );

#ifdef FLIGHT_STATUS_INDICATOR
  flightStatusIndicator ( );
#endif

  if ( shouldProcessRx ( currentTime ) ) {
    processRx ( );
    isRXDataNew = true;

#ifdef BARO
    // the 'annexCode' initialses rcCommand, updateAltHoldState depends on valid rcCommand data.
    if ( haveProcessedAnnexCodeOnce ) {
      if ( sensors ( SENSOR_BARO ) ) {
        updateAltHoldState ( );
      }
    }
#endif

#ifdef SONAR
    // the 'annexCode' initialses rcCommand, updateAltHoldState depends on valid rcCommand data.
    if ( haveProcessedAnnexCodeOnce ) {
      if ( sensors ( SENSOR_SONAR ) ) {
        updateSonarAltHoldState ( );
      }
    }
#endif

  } else {
    // not processing rx this iteration

    executePeriodicTasks ( );

    // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
    // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
    // change this based on available hardware
#ifdef GPS
    if ( feature ( FEATURE_GPS ) ) {
      gpsThread ( );
    }
#endif
  }

  currentTime = micros ( );
  if ( masterConfig.looptime == 0 || ( int32_t ) ( currentTime - loopTime ) >= 0 ) {
    loopTime = currentTime + masterConfig.looptime;

    imuUpdate ( &currentProfile->accelerometerTrims );

    // Measure loop rate just after reading the sensors
    currentTime  = micros ( );
    cycleTime    = ( int32_t ) ( currentTime - previousTime );
    previousTime = currentTime;

#ifdef ENABLE_ACROBAT
    if ( ARMING_FLAG ( ARMED ) ) {
      if ( flipState >= 1 ) {
        flip ( true );
      }

    } else {
      flip ( false );
    }
#endif

    // Gyro Low Pass
    if ( currentProfile->pidProfile.gyro_cut_hz ) {
      int axis;
      static filterStatePt1_t gyroADCState [ XYZ_AXIS_COUNT ];

      for ( axis = 0; axis < XYZ_AXIS_COUNT; axis++ ) {
        gyroADC [ axis ] = filterApplyPt1 ( gyroADC [ axis ], &gyroADCState [ axis ], currentProfile->pidProfile.gyro_cut_hz );
      }
    }

#ifdef UWB
    if ( localisationType == UWB )
      uwbUpdate ( );
#endif

    annexCode ( );

    if ( masterConfig.rxConfig.rcSmoothing ) {
      filterRc ( );
    }

    userCode ( );



    /* used in localisation
     if (command_verify()){

     command_next();

     }

     command_run(currentTime);
     */

#if defined( BARO ) || defined( SONAR )
    haveProcessedAnnexCodeOnce = true;
#endif

#ifdef MAG
    if ( sensors ( SENSOR_MAG ) ) {
      updateMagHold ( );
    }
#endif

#ifdef OPTIC_FLOW
    selectVelOrPosmode ( );
#endif


#ifdef GTUNE
    updateGtuneState ( );
#endif

#if defined( BARO ) || defined( SONAR )
    if ( sensors ( SENSOR_BARO ) || sensors ( SENSOR_SONAR ) ) {
      if ( FLIGHT_MODE ( BARO_MODE ) || FLIGHT_MODE ( SONAR_MODE ) ) {
        applyAltHold ( &masterConfig.airplaneConfig );
      }
    }
#endif

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if ( isUsingSticksForArming ( ) && rcData [ THROTTLE ] <= masterConfig.rxConfig.mincheck
#ifndef USE_QUAD_MIXER_ONLY
         && ! ( ( masterConfig.mixerMode == MIXER_TRI || masterConfig.mixerMode == MIXER_CUSTOM_TRI ) && masterConfig.mixerConfig.tri_unarmed_servo ) && masterConfig.mixerMode != MIXER_AIRPLANE && masterConfig.mixerMode != MIXER_FLYING_WING
#endif
    ) {
      rcCommand [ YAW ] = 0;
    }

    if ( currentProfile->throttle_correction_value && ( FLIGHT_MODE ( ANGLE_MODE ) || FLIGHT_MODE ( HORIZON_MODE ) ) ) {
      rcCommand [ THROTTLE ] += calculateThrottleAngleCorrection ( currentProfile->throttle_correction_value );
    }

#ifdef GPS
    if ( sensors ( SENSOR_GPS ) ) {
      if ( ( FLIGHT_MODE ( GPS_HOME_MODE ) || FLIGHT_MODE ( GPS_HOLD_MODE ) ) && STATE ( GPS_FIX_HOME ) ) {
        updateGpsStateForHomeAndHoldMode ( );
      }
    }
#endif

    // PID - note this is function pointer set by setPIDController()
    pid_controller ( &currentProfile->pidProfile, currentControlRateProfile, masterConfig.max_angle_inclination, &currentProfile->accelerometerTrims, &masterConfig.rxConfig );

    mixTable ( );

#ifdef USE_SERVOS
    //   filterServos();
    //   writeServos();
#endif

    if ( motorControlEnable ) {
      writeMotors ( );
    }

#ifdef BLACKBOX
    if ( ! cliMode && feature ( FEATURE_BLACKBOX ) ) {
      handleBlackbox ( );
    }
#endif

    /*
     #ifdef BAROO
     // the 'annexCode' initialses rcCommand, updateAltHoldState depends on valid rcCommand data.
     if (haveProcessedAnnexCodeOnce) {
     if (sensors(SENSOR_BARO)) {
     updateAltHoldState();
     }
     }
     #endif
     */

    failsafeOnCrash ( );

    executeCommand ( );
  }

#ifdef TELEMETRY
  if ( ! cliMode && feature ( FEATURE_TELEMETRY ) ) {
    telemetryProcess ( &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle );
  }
#endif

#ifdef LED_STRIP
  if ( feature ( FEATURE_LED_STRIP ) ) {
    updateLedStrip ( );
  }
#endif

#ifdef OPTIC_FLOW
  if ( runUserCode ) {

    isLocalisationOn = true;

  } else {

    isLocalisationOn = false;
  }
#endif
}
