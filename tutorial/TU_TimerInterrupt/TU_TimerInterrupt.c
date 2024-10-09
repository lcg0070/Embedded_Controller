/*----------------------------------------------------------------\
Author           : Lee ChanKeun
Created          : 10-04-2024
Modified         : 10-04-2024
Language/ver     : C in CLION with platformio

Description      : Tutorial_Timerinterrupt
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
    setup();
    TIM_TypeDef* timerx;
    timerx = TIM2;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    timerx->PSC   =   840 - 1;		                // Timer counter clock: 1MHz(1us)
    timerx->ARR   =   100 - 1;				    // Set auto reload register to maximum (count up to 65535)
    timerx->DIER |= 1 << 0;                    	// Enable Interrupt
    timerx->CR1  |= 1 << 0;                     	// Enable counter

    NVIC_SetPriority(TIM2_IRQn,2);               	// TIM2_IRQHandler Set priority as 2
    NVIC_EnableIRQ(TIM2_IRQn);			// TIM2_IRQHandler Enable

    // Inifinite Loop ----------------------------------------------------------
    while(1) {}
}

// Initialiization
void setup(void)
{
    RCC_PLL_init();                       // System Clock = 84MHz
    GPIO_init(LED_pin, OUTPUT);      // calls RCC_GPIOA_enable()
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, PULL_UP);
    GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
}

void TIM2_IRQHandler(void){
    if((TIM2->SR & TIM_SR_UIF) == 1){ // update interrupt flag
        if (count > 1000) {
            LED_toggle();
            count = 0;
        }
        count++;
        TIM2->SR &=  ~TIM_SR_UIF;                   // clear by writing 0
    }
}