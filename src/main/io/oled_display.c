/*******************************************************************************
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: io                                                                #
 #  File: \ssd1306.c                                                           #
 #  Created Date: Sat, 25th Jan 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Mon, 27th Jan 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "oled_display.h"

#include "platform.h"

#include "common/printf.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/runtime_config.h"

#include "drivers/sensor.h"
#include "drivers/display_ug2864hsweg01.h"
#include "drivers/system.h"
#include "drivers/accgyro.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "io/rc_controls.h"

#include "rx/rx.h"

#include "version.h"

// Buffer for storing text to be displayed on the OLED
char buffer [ 20 ];

// Pointer to a message string, initialized to NULL (no message)
const char *message = NULL;

// Timing variables
unsigned long updatedMillis            = 0;    // Stores current time in milliseconds
unsigned long lastOledStartUpMillis    = 0;    // Time when OLED startup was initiated
unsigned long lastOledClearMillis      = 0;    // Unused in this code snippet
unsigned long OldOledDisplayDataMillis = 0;    // Last time display data was updated

// Constants for timing intervals
const int OledStartUpTime = 2000;    // Duration for the OLED startup screen
const int OledClearTime   = 500;     // Unused in this code snippet

// Stopwatch timing variables
unsigned long SwStartMillis, SwElapsedMillis;

// State flags
bool devmode            = false;    // Development mode flag
bool StopWatchRunning   = false;    // Tracks if the stopwatch is running
bool OledInitStatus     = false;    // Status of OLED initialization
bool devModeLastState   = false;    // Previous state of development mode
bool OledStartupPageEnd = false;    // Indicates if the startup page has ended

// Blinking control variables
int blink_at      = 2;    // Blink interval
int blink_counter = 0;    // Counter for blinking logic

/**
 * @brief Initializes the OLED display.
 */
void OledStartUpInit ( void ) {
  OledInitStatus        = ug2864hsweg01InitI2C ( );    // Initialize OLED via I2C
  lastOledStartUpMillis = millis ( );                  // Record the start time
}

/**
 * @brief Displays the startup page on the OLED.
 */
void OledStartUpPage ( void ) {
  if ( OledInitStatus ) {
    updatedMillis = millis ( );                                           // Update current time
    if ( updatedMillis - lastOledStartUpMillis < OledStartUpTime ) {      // Check if startup time hasn't elapsed
      i2c_OLED_set_xy ( 2, 0 );
      tfp_sprintf ( buffer, "%s for %s", FwName, targetName );            // Display firmware and target name
      i2c_OLED_send_string ( buffer );
      i2c_OLED_set_xy ( 2, 2 );
      i2c_OLED_send_string ( "By DRONA AVIATION" );                       // Display author information
      i2c_OLED_set_xy ( 0, 4 );
      tfp_sprintf ( buffer, "FW:%s | API:%s", FwVersion, ApiVersion );    // Display firmware and API version
      i2c_OLED_send_string ( buffer );
      i2c_OLED_set_xy ( 2, 6 );
      tfp_sprintf ( buffer, "Build : %s", buildDate );                    // Display build date
      i2c_OLED_send_string ( buffer );
      OledStartupPageEnd = true;                                          // Mark that startup page has been shown
    } else if ( OledStartupPageEnd ) {                                    // Clear the display after showing the startup page
      i2c_OLED_clear_display_quick ( );
      OledStartupPageEnd = false;
    }
  }
}

/**
 * @brief Displays the battery voltage on the OLED.
 */
void dispVoltage ( void ) {
  i2c_OLED_set_xy ( 16, 0 );
  tfp_sprintf ( buffer, "| %d.%d", ( int ) ( vbat / 10 ), ( int ) ( vbat % 10 ) );    // Format voltage
  i2c_OLED_send_string ( buffer );
}

/**
 * @brief Displays whether the device is in development mode.
 */
void dispDevMode ( void ) {
  i2c_OLED_set_xy ( 0, 0 );
  i2c_OLED_send_string ( devmode ? "DEV |" : "    |" );    // Display DEV if in development mode
}

/**
 * @brief Displays the connection status of the receiver.
 */
void dispRxConnection ( void ) {
  i2c_OLED_set_xy ( 0, 7 );
  i2c_OLED_send_string ( ( rxIsReceivingSignal ( ) || ppmIsRecievingSignal ( ) ) ? "RX  |" : "    |" );    // Display RX if receiving signal
}

/**
 * @brief Displays arm status.
 */
void dispArmData ( void ) {
  i2c_OLED_set_xy ( 16, 7 );
  i2c_OLED_send_string ( IS_RC_MODE_ACTIVE ( BOXARM ) ? "| ARM" : "|    " );    // Display ARM if armed
}


/**
 * @brief Displays the flight status on the OLED.
 *
 * This function updates and displays the current flight status message on the OLED screen
 * based on various conditions such as calibration, battery status, signal loss, and arming status.
 * It uses a blinking effect to periodically refresh the message.
 */
void dispFlightStatus ( void ) {
  // Check if it's time to update the message
  if ( blink_counter == blink_at ) {
    if ( status_FSI ( Accel_Gyro_Calibration ) )
      message = " ACC CAL ";    // Accelerometer and Gyroscope Calibration
    else if ( status_FSI ( Mag_Calibration ) )
      message = " MAG CAL ";    // Magnetometer Calibration
    else if ( status_FSI ( LowBattery_inFlight ) || status_FSI ( Low_battery ) )
      message = " LOW BAT ";    // Low Battery Warning
    else if ( status_FSI ( Signal_loss ) || ! ( rxIsReceivingSignal ( ) || ppmIsRecievingSignal ( ) ) )
      message = " RX LOSS ";    // Signal Loss Warning
    else if ( status_FSI ( Crash ) )
      message = "  CRASH  ";    // Crash Detected
    else if ( status_FSI ( Not_ok_to_arm ) )
      message = "NOT READY";    // Not Ready to Arm
    else if ( status_FSI ( Ok_to_arm ) )
      message = "  READY  ";    // Ready to Arm
    else if ( status_FSI ( Armed ) )
      message = "IN FLIGHT";    // Armed and In Flight
  } else if ( blink_counter == 5 ) {
    // Reset message for blinking effect
    message       = "         ";
    blink_counter = 0;
  }

  // Only update display if there is a message
  if ( message ) {
    i2c_OLED_set_xy ( 6, 0 );
    i2c_OLED_send_string ( message );
  }

  blink_counter++;
}


/**
 * @brief Manages and displays the stopwatch on the OLED.
 *
 * This function handles the logic for starting, stopping, and updating a stopwatch based on
 * the RC mode status. It calculates the elapsed time in minutes and seconds and displays it
 * on the OLED screen.
 */
void dispStopWatch ( void ) {
  unsigned long currentMillis = millis ( );    // Get current time

  // Check if RC mode is active
  if ( IS_RC_MODE_ACTIVE ( BOXARM ) ) {
    if ( ! StopWatchRunning ) {
      // Start the stopwatch
      SwElapsedMillis = 0;    // Reset elapsed time
      if ( status_FSI ( Armed ) ) {
        StopWatchRunning = true;
        SwStartMillis    = currentMillis;
      }
    } else {
      // Update elapsed time
      SwElapsedMillis = currentMillis - SwStartMillis;
    }
  } else if ( StopWatchRunning ) {
    // Stop the stopwatch
    StopWatchRunning = false;
    SwElapsedMillis  = currentMillis - SwStartMillis;
  }

  // Calculate minutes and seconds from elapsed milliseconds
  unsigned long totalSeconds = SwElapsedMillis / 1000;    // Convert millis to seconds
  unsigned long minutes      = totalSeconds / 60;
  unsigned long seconds      = totalSeconds % 60;

  // Display minutes with leading zero on OLED
  i2c_OLED_set_xy ( 7, 7 );
  tfp_sprintf ( buffer, "%02lu :", minutes );
  i2c_OLED_send_string ( buffer );

  // Display seconds with leading zero on OLED
  i2c_OLED_set_xy ( 12, 7 );
  tfp_sprintf ( buffer, "%02lu", seconds );
  i2c_OLED_send_string ( buffer );
}


/**
 * @brief Displays navigation-related information on the OLED screen.
 *
 * This function checks if the startup page has ended. If so, it proceeds to display various
 * navigation and system status elements such as device mode, flight status, voltage,
 * receiver connection status, stopwatch, and arm data on the OLED screen.
 */
void OledDisplayNav ( void ) {
  // Ensure startup page has ended before displaying navigation information
  if ( ! OledStartupPageEnd ) {
    // Display device mode
    dispDevMode ( );
    // Display flight status
    dispFlightStatus ( );
    // Display voltage level
    dispVoltage ( );
    // Display receiver connection status
    dispRxConnection ( );
    // Display stopwatch timer
    dispStopWatch ( );
    // Display arm data
    dispArmData ( );
  }
}



/**
 * @brief Displays system data on the OLED screen.
 *
 * This function checks if the device is not in development mode and the startup page has ended.
 * It then displays pitch, roll, yaw, and throttle values on the OLED screen. If the development
 * mode state changes, it clears the display to update the information.
 */
void OledDisplaySystemData ( void ) {
  // Check if not in dev mode and startup page has ended
  if ( ! devmode && ! OledStartupPageEnd ) {
    // Display pitch value
    i2c_OLED_set_xy ( 1, 2 );
    tfp_sprintf ( buffer, "P : %d   ", ( int ) ( inclination.values.pitchDeciDegrees ) / 10 );
    i2c_OLED_send_string ( buffer );

    // Display roll value
    i2c_OLED_set_xy ( 1, 3 );
    tfp_sprintf ( buffer, "R : %d   ", ( int ) ( inclination.values.rollDeciDegrees ) / 10 );
    i2c_OLED_send_string ( buffer );

    // Display yaw value
    i2c_OLED_set_xy ( 1, 4 );
    tfp_sprintf ( buffer, "Y : %d   ", ( int ) ( heading ) );
    i2c_OLED_send_string ( buffer );

    // Display RC channel pitch
    i2c_OLED_set_xy ( 11, 2 );
    tfp_sprintf ( buffer, "p : %d   ", ( int ) ( rcData [ 1 ] ) );
    i2c_OLED_send_string ( buffer );

    // Display RC channel roll
    i2c_OLED_set_xy ( 11, 3 );
    tfp_sprintf ( buffer, "R : %d   ", ( int ) ( rcData [ 0 ] ) );
    i2c_OLED_send_string ( buffer );

    // Display RC channel yaw
    i2c_OLED_set_xy ( 11, 4 );
    tfp_sprintf ( buffer, "Y : %d   ", ( int ) ( rcData [ 2 ] ) );
    i2c_OLED_send_string ( buffer );

    // Display RC channel throttle
    i2c_OLED_set_xy ( 11, 5 );
    tfp_sprintf ( buffer, "T : %d   ", ( int ) ( rcData [ 3 ] ) );
    i2c_OLED_send_string ( buffer );
  }

  // Clear display if development mode state has changed
  if ( devModeLastState != devmode ) {
    i2c_OLED_clear_display_quick ( );
  }

  // Update last known dev mode state
  devModeLastState = devmode;
}

/**
 * @brief Updates and displays navigation and system data on the OLED.
 *
 * This function periodically updates the display with navigation and system data,
 * ensuring that the display refreshes at a set interval.
 */
void OledDisplayData ( void ) {
  if ( OledInitStatus ) {
    // Check if enough time has passed since the last display update
    if ( updatedMillis - OldOledDisplayDataMillis >= 200 ) {
      // Display navigation-related information
      OledDisplayNav ( );
      // Display system data
      OledDisplaySystemData ( );
      // Update the last display update timestamp
      OldOledDisplayDataMillis = updatedMillis;
    }
  }
}