/*
 * ppm_test.h
 *
 *  Created on: 21-May-2022
 *      Author: Drona
 */




/*
 * PPM-User.h
 *
 *  Created on: 28-Feb-2022
 *      Author: Tech Tronix Projects
 */

#pragma once

#include "PlutoPilot.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef enum {

     MSP_IP,
	 PPM_IP
} RX_state;

class RX_P {
public:
    void InputStatus(RX_state STATE);

};

extern bool PPM_MODE;
extern RX_P RX;
#ifdef __cplusplus
}
#endif





