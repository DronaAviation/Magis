



/*
 * RxMode.cpp
 *
 *  Created on: 21-May-2022
 *      Author: Tech Tronix Projects
 */


#include"RxMode.h"

void RX_P::InputStatus(RX_state STATE)
{

    switch (STATE) {

    case MSP_IP:

    	PPM_MODE=0;

        break;

    case PPM_IP:

    	PPM_MODE=1;

        break;
    default:
    	PPM_MODE=0;
    }

}
bool PPM_MODE=0;
RX_P RX;

