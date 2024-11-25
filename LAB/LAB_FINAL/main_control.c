// /*----------------------------------------------------------------\
// Author           : Lee ChanKeun
// Created          : 11-19-2024
// Modified         : 11-19-2024
// Language/ver     : C in CLION with platformio
//
// Description      : LAB_Final_Main
// /----------------------------------------------------------------*/

#include <math.h>
#include "smartFarm_Hani.h"
#include "stm32f411xe.h"
#include "ecSTM32F4.h"


int temp = 0;

uint8_t btData = 0;

void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1) {
        if(GPIO_read(PA_7)) {
            GPIO_write(LED_PIN,1);
        }else {
            GPIO_write(LED_PIN,0);
        }

    }
}

// Initialiization
void setup(void){
    RCC_PLL_init();                 // System Clock = 84MHz
    SysTick_init();			         // SysTick Init

    // each setup
    // UART1_setup();
    LED_setup();
    GPIO_init(PA_7, INPUT);
    GPIO_otype(PA_7, OUTPUT_PUSH_PULL);
    GPIO_pupd(PA_7, NO_PULLUP_PULLDOWN);
    GPIO_ospeed(PA_7, MEDIUM_SPEED);
}




// =======================================
// interrupt
// =======================================


// void USART1_IRQHandler() {  // USART1 interrupt handler
//      if (is_USART_RXNE(USART1)) {
//          int arrow_flag = 0;
//          btData = USART_read(USART1);
//          if (btData == 1) GPIO_write(LED_PIN, temp);
//          printf("%c", btData); // TX to USART2(PC)
//      }
//  }
