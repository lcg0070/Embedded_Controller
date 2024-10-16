//
// Created by 이찬근 on 2024. 10. 8..
//

#ifndef ECPWM2_H
#define ECPWM2_H

#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecTIM2.h"
#include "ecPinNames.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

     /* PWM Configuration using PinName_t Structure */

     /* PWM initialization */
     // Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
     void PWM_init(PinName_t pinName, uint8_t sec_flag, uint32_t sec);
     void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);


     /* PWM PERIOD SETUP */

     void PWM_period(PinName_t pinName, uint8_t sec_flag, uint32_t sec);
     void PWM_period_ms(PinName_t pinName,  uint32_t msec);	// allowable range for msec:  1~2,000
     void PWM_period_us(PinName_t pinName, uint32_t usec); // allowable range for usec:  1~1,000


     /* DUTY RATIO SETUP */
     // High Pulse width in msec
     void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms);
     void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);  // same as void PWM_pulsewidth
     void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us);
     // Duty ratio 0~1.0
     void PWM_duty(PinName_t pinName, float duty);


#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif //ECPWM2_H
