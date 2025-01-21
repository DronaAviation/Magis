/*******************************************************************************
 #  Copyright (c) 2025 DRONA AVIATION                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: API                                                               #
 #  File: \RxConfig.cpp                                                        #
 #  Created Date: Tue, 26th Jan 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 21st Jan 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/



#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


#include "platform.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/serial.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/rc_controls.h"


#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"


#include "config/runtime_config.h"
#include "config/config.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#include "PlutoPilot.h"
#include "RxConfig.h"


// Global variables to store device mode configuration
uint8_t DevModeAUX       = 0;    // Auxiliary channel for device mode
uint16_t DevModeMinRange = 0;    // Minimum range for device mode
uint16_t DevModeMaxRange = 0;    // Maximum range for device mode

rx_mode_e RxMode = Rx_ESP;

/**
 * Configures auxiliary channels for a specific flight mode.
 *
 * @param flightMode The flight mode to configure (e.g., Mode_ARM, Mode_ANGLE).
 * @param rxChannel The receiver channel associated with the mode.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Rc_Rx_Config_P::configAUX ( flight_mode flightMode, rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  switch ( flightMode ) {
    case Mode_ARM:
      currentProfile->modeActivationConditions [ 0 ] = { ( boxId_e ) BOXARM, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    case Mode_ANGLE:
      currentProfile->modeActivationConditions [ 1 ] = { ( boxId_e ) BOXANGLE, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    case Mode_BARO:
      currentProfile->modeActivationConditions [ 2 ] = { ( boxId_e ) BOXBARO, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    case Mode_MAG:
      currentProfile->modeActivationConditions [ 3 ] = { ( boxId_e ) BOXMAG, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    case Mode_HEADFREE:
      currentProfile->modeActivationConditions [ 4 ] = { ( boxId_e ) BOXHEADFREE, rxChannel - NON_AUX_CHANNEL_COUNT, CHANNEL_VALUE_TO_STEP ( minRange ), CHANNEL_VALUE_TO_STEP ( maxRange ) };
      break;
    default:
      break;
  }
}

bool AuxChangeEnable = false;

/**
 * Configures the receiver mode and sets up auxiliary channels accordingly.
 *
 * @param rxMode The receiver mode to configure (e.g., Rx_ESP, Rx_PPM).
 */
void Rc_Rx_Config_P::rxMode ( rx_mode_e rxMode ) {
  switch ( rxMode ) {
    case Rx_ESP:
      AuxChangeEnable = false;
      featureSet ( FEATURE_RX_MSP );
      featureClear ( FEATURE_RX_SERIAL );
      featureClear ( FEATURE_RX_PARALLEL_PWM );
      featureClear ( FEATURE_RX_PPM );

      // Configure auxiliary channels for various modes
      configAUX ( Mode_ARM, Rx_AUX4, 1300, 2100 );
      configAUX ( Mode_ANGLE, Rx_AUX4, 900, 2100 );
      configAUX ( Mode_BARO, Rx_AUX3, 1300, 2100 );
      configAUX ( Mode_MAG, Rx_AUX1, 900, 1300 );
      configAUX ( Mode_HEADFREE, Rx_AUX1, 1300, 1700 );
      DevModeAUX      = Rx_AUX2;    // Set the auxiliary channel
      DevModeMinRange = 1450;       // Set the minimum range
      DevModeMaxRange = 1550;       // Set the maximum range
      break;

    case Rx_PPM:
      AuxChangeEnable = true;

      featureSet ( FEATURE_RX_PPM );
      featureClear ( FEATURE_RX_SERIAL );
      featureClear ( FEATURE_RX_PARALLEL_PWM );
      featureClear ( FEATURE_RX_MSP );

      // Configure auxiliary channels for various modes
      configAUX ( Mode_ARM, Rx_AUX2, 1300, 2100 );
      configAUX ( Mode_ANGLE, Rx_AUX2, 900, 2100 );
      configAUX ( Mode_BARO, Rx_AUX3, 1300, 2100 );
      configAUX ( Mode_MAG, Rx_AUX1, 900, 1300 );
      configAUX ( Mode_HEADFREE, Rx_AUX1, 1300, 1700 );
      configureDevMode ( Rx_AUX4, 1500, 2100 );
      break;

    default:
      break;
  }
}

/**
 * Configures ARM mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Rc_Rx_Config_P::configureArmMode ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    configAUX ( Mode_ARM, rxChannel, minRange, maxRange );
  }
}

/**
 * Configures ANGLE mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Rc_Rx_Config_P::configureModeANGLE ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    configAUX ( Mode_ANGLE, rxChannel, minRange, maxRange );
  }
}

/**
 * Configures BARO mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Rc_Rx_Config_P::configureModeBARO ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    configAUX ( Mode_BARO, rxChannel, minRange, maxRange );
  }
}

/**
 * Configures MAG mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Rc_Rx_Config_P::configureMagMode ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    configAUX ( Mode_MAG, rxChannel, minRange, maxRange );
  }
}

/**
 * Configures HEADFREE mode if auxiliary change is enabled.
 *
 * @param rxChannel The receiver channel to configure.
 * @param minRange The minimum range value for activation.
 * @param maxRange The maximum range value for activation.
 */
void Rc_Rx_Config_P::configureHeadfreeMode ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  if ( AuxChangeEnable ) {
    configAUX ( Mode_HEADFREE, rxChannel, minRange, maxRange );
  }
}



/**
 * @brief Configures the device mode with specified parameters.
 *
 * This function sets the auxiliary channel, minimum range, and maximum range
 * for the device mode configuration. The configuration is applied only if
 * AuxChangeEnable is true.
 *
 * @param rxChannel The auxiliary channel to be used for device mode.
 * @param minRange The minimum range value for the device mode.
 * @param maxRange The maximum range value for the device mode.
 */
void Rc_Rx_Config_P::configureDevMode ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange ) {
  // Check if change in auxiliary settings is enabled
  if ( AuxChangeEnable ) {
    DevModeAUX      = rxChannel;    // Set the auxiliary channel
    DevModeMinRange = minRange;     // Set the minimum range
    DevModeMaxRange = maxRange;     // Set the maximum range
  }
}


Rc_Rx_Config_P Receiver;