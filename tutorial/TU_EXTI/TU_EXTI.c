/*----------------------------------------------------------------\
Author           : Lee ChanKeun
Created          : 09-26-2024
Modified         : 09-26-2024
Language/ver     : C in CLION with platformio

Description      : Tutorial_EXTI
/----------------------------------------------------------------*/



//#include "ecSTM32F4v2.h"
#include "ecRCC2.h"
#include "ecGPIO2.h"

#define LED_PIN   PA_5
#define BUTTON_PIN PC_13

#define SYSCFG_EN 14

void setup(void);

int led_state = 1;
void LED_toggle() {
    GPIO_write(LED_PIN, !led_state);
    led_state = !led_state;
}

int main(void) {

    // System CLOCK, GPIO Initialiization ----------------------------------------
    setup();


    // EXTI Initialiization ------------------------------------------------------

    // SYSCFG peripheral clock enable
    RCC->APB2ENR |= 1UL<<SYSCFG_EN; //SYSCFG_EN = 14

    // Connect External Line to the GPIO
    // Button: PC_13 -> EXTICR3(EXTI13)
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= 0b0010UL<<4;

    // Falling trigger enable (Button: pull-up)
    EXTI->FTSR |= 1UL << 13;

    // Unmask (Enable) EXT interrupt
    EXTI->IMR |= 1UL << 13;

    // Interrupt IRQn, Priority
    NVIC_SetPriority(EXTI15_10_IRQn, 0);  		// Set EXTI priority as 0
    NVIC_EnableIRQ(EXTI15_10_IRQn); 			// Enable EXTI


    while (1);
}


void EXTI15_10_IRQHandler(void) {
    if ((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13) {
        LED_toggle();
        EXTI->PR |= EXTI_PR_PR13; // cleared by writing '1'
    }
}

// Initialiization
void setup(void)
{
    RCC_PLL_init();                         // System Clock = 84MHz
    // Initialize GPIOA_5 for Output
    GPIO_init(LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, NO_PULLUP_PULLDOWN);


    // Initialize GPIOC_13 for Input Button
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    GPIO_pupd(BUTTON_PIN, NO_PULLUP_PULLDOWN);
}