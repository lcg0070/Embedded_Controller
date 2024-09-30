/**
  ******************************************************************************
  * @author  LeeChanKeun
  * @Mod		 2024-09-06 by LCK
  * @brief   Embedded Controller:  Tutorial Digital Out
  *					 - Turn on LED LD2
  *
  ******************************************************************************
*/


// GPIO Mode			 : Input(00), Output(01), AlterFunc(10), Analog(11, reset)
// GPIO Speed			 : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
// GPIO Push-Pull	 : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)


#include "stm32f4xx.h"
#include "ecRCC2.h"

#define LED_PIN    5		//LD2


int main(void) {
    /* Part 1. RCC GPIOA Register Setting */
    RCC_HSI_init();
    RCC_GPIOA_enable();

    /* Part 2. GPIO Register Setting */
    // GPIO Mode Register
    GPIOA->MODER &= ~(0b11<<LED_PIN*2);											// Clear '00' for Pin 5
    GPIOA->MODER |= 0b01<<(LED_PIN*2); 											// Set '01' for Pin 5

    // GPIO Output Type Register
    GPIOA->OTYPER &= ~(0b1<<LED_PIN);											// Clear '00'
    GPIOA->OTYPER |= 0b0<<(LED_PIN);											// 0:Push-Pull

    // GPIO Pull-Up/Pull-Down Register
    GPIOA->PUPDR  &= ~(0b11<<(LED_PIN*2));										// 00: none

    // GPIO Output Speed Register
    GPIOA->OSPEEDR &= ~(0b11 <<(2*LED_PIN));
    GPIOA->OSPEEDR |= 	0b10 <<(2 * LED_PIN);									//10:Fast Speed

    // Dead loop & program hangs here
    while(1){
        //	 GPIOA->ODR = 1UL << LED_PIN; 	// Set LED_PIN = H, others=L
        GPIOA->ODR |= (0b01 << LED_PIN);	 		// Change only LED_PIN = H
    }
}