/*******************************************************************************
 #  Copyright (c) 2025 DRONA AVIATION                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: API                                                               #
 #  File: \RxConfig.h                                                          #
 #  Created Date: Tue, 16th Jan 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Tue, 21st Jan 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/







#pragma once


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  Rx_ESP,
  Rx_PPM
} rx_mode_e;

typedef enum {
  Mode_ARM,
  Mode_ANGLE,
  Mode_BARO,
  Mode_MAG,
  Mode_HEADFREE
} flight_mode;

typedef enum {
  Rx_AUX1 = 4,
  Rx_AUX2,
  Rx_AUX3,
  Rx_AUX4,
  Rx_AUX5,
} rx_channel_e;


extern uint8_t DevModeAUX;
extern uint16_t DevModeMinRange;
extern uint16_t DevModeMaxRange;


class Rc_Rx_Config_P {
 private:
  void configAUX ( flight_mode flightMode, rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
  

 public:
  void rxMode ( rx_mode_e rxMode );
  void configureArmMode ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
  void configureModeANGLE ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
  void configureModeBARO ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
  void configureMagMode ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
  void configureHeadfreeMode ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
  void configureDevMode ( rx_channel_e rxChannel, uint16_t minRange, uint16_t maxRange );
};

extern Rc_Rx_Config_P Receiver;

#ifdef __cplusplus
}
#endif
