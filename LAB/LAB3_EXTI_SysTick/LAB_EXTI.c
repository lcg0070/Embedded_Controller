/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lee ChanKeun
Created          : 10-01-2024
Modified         : 10-01-2024
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

    // Inifinite Loop ---------------------------------------------------------
    while(1){}
}

// Interrupt(Button input) ----------------------------------------------------
void EXTI15_10_IRQHandler(void) {
    //check pending
    if(is_pending_EXTI(BUTTON_PIN) ) {
        //debouncing
        for(int i=0; i<30000; i++){}

        //displaying function
        sevensegment_display(cnt % 10);
        cnt++;
        if (cnt > 9) cnt = 0;

        //clear pending
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