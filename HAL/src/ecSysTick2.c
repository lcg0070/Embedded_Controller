/*
******************************************************************************
* @author  LeeChanKeun
* @Mod		 2024-09-27 by LCK
* @brief   Embedded Controller:  EC_SysTick
*
******************************************************************************
*/

#include "ecSysTick2.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

volatile uint32_t msTicks;

//EC_SYSTEM_CLK

void SysTick_init(void){

    //  SysTick Control and Status Register
    SysTick->CTRL = 0;											// Disable SysTick IRQ and SysTick Counter

    // Select processor clock
    // 1 = processor clock;  0 = external clock
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    // uint32_t MCU_CLK=EC_SYSTEM_CLK
    // SysTick Reload Value Register
    SysTick->LOAD = MCU_CLK_PLL / 1000 - 1;						// 1ms, for HSI PLL = 84MHz.

    // SysTick Current Value Register
    SysTick_reset();

    // Enables SysTick exception request
    // 1 = counting down to zero asserts the SysTick exception request
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    // Enable SysTick IRQ and SysTick Timer
    SysTick_enable();

    NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
    NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}



void SysTick_Handler(void){
    SysTick_counter();
}

void SysTick_counter(){
    msTicks++;
}

void delay_ms (uint32_t msec){
    uint32_t curTicks;

    curTicks = msTicks;

    while ((msTicks - curTicks) < msec){}
    msTicks = 0;
}



// SysTick Current Value Register
void SysTick_reset(void)
{
    SysTick->VAL = 0;
}

void SysTick_enable(void)
{
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
void SysTick_disable(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

uint32_t SysTick_val(void) {
    return SysTick->VAL;
}
