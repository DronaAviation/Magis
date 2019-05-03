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

#pragma once

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


//typedef enum angles {
//    AG_ROLL = 0,
//    AG_PITCH,
//    AG_YAW,
//
//} angles_e;


typedef enum flight_mode{
    FM_ALTHOLD,
    FM_ANGLE
}flight_mode_e;



class Flight_P {

public:

   // int16_t getAngle(angles_e angle);

    int32_t getEstimatedAltitude(void);

    int32_t getVelocityZ(void);

    void activateFlightMode(flight_mode_e mode);

    void deactivateFlightMode(flight_mode_e mode);

    bool isFlightMode(flight_mode_e mode);

    //void setAngleRate(angles_e angle, uint8_t rate);

   // uint8_t getAngleRate(angles_e angle);

    void setAltholdHeight(int32_t height);

    void setRelativeAltholdHeight(int32_t height);

    int32_t getAltholdHeight(void);

};

extern Flight_P Flight;

#ifdef __cplusplus
}
#endif
