//
// Created by 이찬근 on 2024. 9. 27..
//

#ifndef ECEXTI2_H
#define ECEXTI2_H

#include "ecGPIO2.h"
#include "ecSysTick2.h"

#define FALL 0
#define RISE 1
#define BOTH 2

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

     void EXTI_init(PinName_t pinName, int trig_type,int priority);
     void EXTI_enable(PinName_t pinName);
     void EXTI_disable(PinName_t pinName);
     uint32_t is_pending_EXTI(PinName_t pinName);
     void clear_pending_EXTI(PinName_t pinName);

#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif //ECEXTI2_H
