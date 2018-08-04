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

#include "XGPS.h"

#include "../API-Utils.h"

void XGPS_P::setPosholdPostion(int32_t pos_latitude, int32_t pos_longitude)
{

    user_GPS_coord[0] = pos_latitude;
    user_GPS_coord[1] = pos_longitude;
    isUserGPSCoordSet = true;

}

XGPS_P XGPS;
