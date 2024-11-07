// /*----------------------------------------------------------------\
// Author           : Lee ChanKeun
// Created          : 11-05-2024
// Modified         : 11-05-2024
// Language/ver     : C in CLION with platformio
//
// Description      : LAB_RC_CAR
// /----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "RC_CAR.h"
#include "ecSTM32F4.h"


uint32_t IR_Value[2] = { 0, 0 };
float ultrasonic_distance = 0.0f;

uint8_t mode = MANUAL;
uint8_t btData = 0;

void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1) {;}
}


// Initialiization
void setup(void){
    RCC_PLL_init();                 // System Clock = 84MHz
    SysTick_init();			         // SysTick Init

    // UART1_init();			            // UART1 Init

    // // each setup
    DC_setup();
    ultrasonic_setup();

    ADC_setup();                   // priority 2

}


// =======================================
// interrupt
// =======================================

// main interrupt
void TIM3_IRQHandler(void){
   switch(mode) {
      case MANUAL:
         1;
      break;
      case AUTO:
         1;
      break;
   }
	auto_control(IR_Value, ultrasonic_distance);

}

// IR sensor interrupt
void ADC_IRQHandler(void){
	IR_control(IR_Value);

}

// ultra sonic interrupt
void TIM4_IRQHandler(void){
     ultrasonic_distance = ultrasonic_control();
}



