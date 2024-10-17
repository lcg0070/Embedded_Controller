//
// Created by 이찬근 on 2024. 10. 11..
//

#ifndef ECICAP2_H
#define ECICAP2_H

#include "stm32f411xe.h"
#include "ecPinNames.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/* Input Capture*/

// ICn selection according to CHn
#define FIRST 1
#define SECOND 2

// Edge Type
#define IC_RISE 0
#define IC_FALL 1
#define IC_BOTH 2

// Input Capture Number
#define IC_1    1
#define IC_2    2
#define IC_3    3
#define IC_4    4


/**
 *ICAP_pinmap : initialize channel and pin according to the input pinName
 *  Parameter :
 *      -PinName_t   : input PinName
 *      -TIM_TypeDef : input TIM register
 *      -chN         : input channel number
**/
void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);

 /**
  *ICAP_init : initialize the GPIO, TIM, TIMER register value
  *  Parameter :
  *      -PinName_t   : input PinName
 **/
void ICAP_init(PinName_t pinName);

 /**
 *ICAP_setup : initialize ICAP value(edge type, IC number etc.)
 *  Parameter :
 *      -PinName_t   : input PinName
 *      -ICn         : input channel number
 *      -edge_type   : input edge_type
**/
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);

/**
 *ICAP_counter_us : set the timer value to 1[usec] when pll is 84[Mhz]
 *  Parameter :
 *      -PinName_t   : input PinName
 *      -usec        : input PSC value
**/
void ICAP_counter_us(PinName_t pinName, int usec);


 /**
 *ICAP_capture : capture the ICAP value
 *  Parameter :
 *      -TIMx        : Pin_number's Tim register's value
 *      -ICn         : Input Channel number
**/
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn);


/**
*is_CCIF : Check, if it is CCIF true of not
*  Parameter :
*      -TIMx        : Pin_number's Tim register's value
*      -CCnum       : Timx's CCR register value
**/
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);  // CCnum= 1~4


/**
*clear_CCIF : clear CCIF value
*  Parameter :
*      -TIMx        : Pin_number's Tim register's value
*      -CCnum       : Timx's CCR register value
**/
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);


#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif //ECICAP2_H
