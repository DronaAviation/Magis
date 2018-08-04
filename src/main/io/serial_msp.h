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

#include "io/serial.h"
#include "drivers/serial.h"

// Each MSP port requires state and a receive buffer, revisit this default if someone needs more than 2 MSP ports.
#define MAX_MSP_PORT_COUNT 2

void mspInit(serialConfig_t *serialConfig);

void mspProcess(void);
void sendMspTelemetry(void);
void mspSetTelemetryPort(serialPort_t *mspTelemetryPort);
void mspAllocateSerialPorts(serialConfig_t *serialConfig);
void mspReleasePortIfAllocated(serialPort_t *serialPort);

void serialize8Debug(uint8_t a);
bool mspSerialRxBytesWaiting(void);
uint8_t mspSerialRead(void);

#ifdef __cplusplus
}
#endif 
