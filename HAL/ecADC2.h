//
// Created by 이찬근 on 2024. 10. 29..
//

#ifndef ECADC2_H
#define ECADC2_H

#include "stm32f411xe.h"
#include "ecSTM32F4.h"

// ADC trigmode
#define SW   0
#define TRGO 1

// ADC contmode
#define CONT   0
#define SINGLE 1

// Edge Type
#define RISE_ADC 1
#define FALL_ADC 2
#define BOTH_ADC 3

#define _DEFAULT 0
#define EC_NONE  0x00



/////////////////////////////////////////////////////
// ADC default setting
/////////////////////////////////////////////////////

// ADC init
// Default:  one-channel mode, continuous conversion
// Default: HW trigger - TIM3 counter, 1msec
void ADC_init(PinName_t pinName);
void JADC_init(PinName_t pinName);


// Multi-Channel Scan Sequence
void ADC_sequence(PinName_t *seqCHn, int seqCHnums);
void JADC_sequence(PinName_t *seqCHn, int seqCHnums);

// ADC start
void ADC_start(void);

// flag for ADC interrupt
uint32_t is_ADC_EOC(void);
uint32_t is_ADC_OVR(void);
void clear_ADC_OVR(void);

// read ADC value
uint32_t ADC_read(void);


/////////////////////////////////////////////////////
// Advanced Setting
/////////////////////////////////////////////////////
// Conversion mode change: CONT, SINGLE / Operate both ADC,JADC
void ADC_conversion(int convMode);

void ADC_trigger(TIM_TypeDef* TIMx, int msec, int edge);
void JADC_trigger(TIM_TypeDef* TIMx, int msec, int edge); // JADC setting

// Private Function
void ADC_pinmap(PinName_t pinName, uint32_t *chN);

void clear_ADC_JEOC(void);

void JADC_start(void);
uint32_t is_ADC_JEOC(void);
uint32_t JADC_read(int JDRn);

#endif //ECADC2_H
