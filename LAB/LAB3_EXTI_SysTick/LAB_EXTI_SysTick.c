/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lee ChanKeun
Created          : 10-01-2024
Modified         : 10-01-2024
Language/ver     : C in CLION with platformio

Description      : SysTick(with 7-segment lab)
/----------------------------------------------------------------*/


#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"

void setup(void);

unsigned int cnt = 0;
int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1) {
        sevensegment_display(cnt % 10);
        delay_ms(1000);
        cnt++;
        if (cnt > 9) cnt = 0;
        SysTick_reset();
    }
}

void EXTI15_10_IRQHandler(void) {
    if(is_pending_EXTI(BUTTON_PIN) ) {
        cnt = 0;
        sevensegment_display(cnt % 10);
        clear_pending_EXTI(BUTTON_PIN);
    }
}

// Initialiization
void setup(void)
{
    RCC_PLL_init();
    SysTick_init();
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()

    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
    EXTI_init(BUTTON_PIN, FALL,13);
}