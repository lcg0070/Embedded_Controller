/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lee ChanKeun
Created          : 09-20-2024
Modified         : 09-20-2024
Language/ver     : C in CLION with platformio

Description      : 7-segment display function
/----------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"

#define LED_PIN PA_5
#define BUTTON_PIN PC_13

void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();
    unsigned int cnt = 0;

    // Inifinite Loop ----------------------------------------------------------
    while(1){
        sevensegment_display(cnt % 10);

        if(GPIO_read(BUTTON_PIN) == 0) {
            cnt++;
            while (!GPIO_read(BUTTON_PIN)) {}
        }
        if (cnt > 9) cnt = 0;
    }
}


// Initialiization
void setup(void)
{
    RCC_HSI_init();
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
}