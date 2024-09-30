/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lee ChanKeun
Created          : 09-30-2024
Modified         : 09-30-2024
Language/ver     : C in CLION with platformio

Description      : EXTI(7-segment lab)
/----------------------------------------------------------------*/


#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecEXTI2.h"

void setup(void);

static unsigned int cnt = 0;

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1){}
}

void EXTI15_10_IRQHandler(void) {
    if(is_pending_EXTI(BUTTON_PIN) ) {
        for(int i=0; i<30000; i++){}    //debouncing
        sevensegment_display(cnt % 10);
        cnt++;
        if (cnt > 9) cnt = 0;
        clear_pending_EXTI(BUTTON_PIN);
    }
}

// Initialiization
void setup(void)
{
    RCC_HSI_init();
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
    EXTI_init(BUTTON_PIN, FALL,13);
}