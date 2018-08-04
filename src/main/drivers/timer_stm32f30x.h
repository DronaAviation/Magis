#pragma once

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f30x.h"

void TIM_SelectOCxM_NoDisable(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);

#ifdef __cplusplus
}
#endif 
