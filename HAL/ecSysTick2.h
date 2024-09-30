//
// Created by 이찬근 on 2024. 9. 27..
//

#ifndef ECSYSTICK2_H
#define ECSYSTICK2_H

#include "stm32f4xx.h"
#include "ecRCC2.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

     void SysTick_init(void);
     void SysTick_Handler(void);
     void SysTick_counter();
     void delay_ms(uint32_t msec);
     void SysTick_reset(void);
     void SysTick_enable(void);
     void SysTick_disable (void);
     uint32_t SysTick_val(void);

#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif //ECSYSTICK2_H
