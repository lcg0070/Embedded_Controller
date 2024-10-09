//
// Created by 이찬근 on 2024. 10. 4..
//
/*----------------------------------------------------------------\
Author           : Lee ChanKeun
Created          : 10-04-2024
Modified         : 10-04-2024
Language/ver     : C in CLION with platformio

Description      : Tutorial_TimerPWM
/----------------------------------------------------------------*/
#include <ecEXTI2.h>

#include "stm32f411xe.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"


PinName_t LED_pin = PA_5;
uint32_t count = 0;

int led_state = 1;
void LED_toggle() {
    GPIO_write(LED_PIN, !led_state);
    led_state = !led_state;
}

void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    RCC_PLL_init();                         // System Clock = 84MHz
    SysTick_init();                         // for delay_ms()

    GPIO_init(LED_PIN, AF);     // GPIOA 5 ALTERNATE function
    GPIO_ospeed(LED_PIN, HIGH_SPEED);   // GPIOA 5 HIGH SPEED

    // TEMP: TIMER Register Initialiization --------------------------------------------------------
    TIM_TypeDef *TIMx;
    TIMx = TIM2;

    // GPIO: ALTERNATIVE function setting
    // GPIOA->AFR[0]	 =           						// AF1 at PA5 = TIM2_CH1 (p.150)
    GPIOA->AFR[0] &= ~(0b1111)<<4*5;
    GPIOA->AFR[0] |=  (0b0001)<<4*5;



    // TIMER: PWM setting
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;         // Enable TIMER clock

    TIMx->CR1 |= 	1 << 0;			            // Direction Up-count
    TIMx->PSC = 	840 - 1;					// Set Timer CLK = 100kHz : (PSC + 1) = 84MHz/100kHz --> PSC = ?
    TIMx->ARR = 	100 - 1;				    // Auto-reload: Upcounting (0...ARR).// Set Counter CLK = 1kHz : (ARR + 1) = 100kHz/1kHz --> ARR = ?

    TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;  			// Clear ouput compare mode bits for channel 1
    TIMx->CCMR1 |= (0b110 << 4);                // OC1M = 110 for PWM Mode 1 output on ch1
    TIMx->CCMR1	|= TIM_CCMR1_OC1PE;    		    // Output 1 preload enable (make CCR1 value changable)

    TIMx->CCR1 = (TIMx->ARR+1)/2;      			// Output Compare Register for channel 1

    TIMx->CCER &= ~TIM_CCER_CC1P;    			// select output polarity: active high
    TIMx->CCER |= TIM_CCER_CC1E;				// Enable output for ch1

    TIMx->CR1  |= TIM_CR1_CEN;      			// Enable counter

    // Inifinite Loop ----------------------------------------------------------
    while(1){
        //Create the code to change the brightness of LED as 10kHZ (use "delay(1000)")
        for(int i =0;i<10;i++){
            TIM2->CCR1 = i*10;
            delay_ms(100);
        }
        for(int i =0;i<10;i++){
            TIM2->CCR1 = (10-i)*10;
            delay_ms(100);
        }
    }
}


// Initialiization
void setup(void)
{
    RCC_PLL_init();                       // System Clock = 84MHz
    SysTick_init();
    GPIO_init(LED_pin, OUTPUT);      // calls RCC_GPIOA_enable()
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, NO_PULLUP_PULLDOWN);
    GPIO_ospeed(LED_PIN, HIGH_SPEED);
}

