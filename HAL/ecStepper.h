//
// Created by 이찬근 on 2024. 10. 22..
//

#ifndef ECSTEPPER_H
#define ECSTEPPER_H


#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecSysTick2.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

     //State mode
#define HALF 0
#define FULL 1

#define DIR_CCW 1
#define DIR_CW 0

 /* Stepper Motor */
 //stepper motor function
 typedef struct{
     PinName_t pins[4];
     uint32_t _step_num;
 } Stepper_t;



 void Stepper_init(PinName_t pinNames[]);


 void Stepper_setSpeed(long whatSpeed);


 void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode);


 void Stepper_stop(void);



#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif //ECSTEPPER_H
