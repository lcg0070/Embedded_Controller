/*----------------------------------------------------------------\
Author           : Lee ChanKeun
Created          : 09-26-2024
Modified         : 09-26-2024
Language/ver     : C in CLION with platformio

Description      : Tutorial_EXTI
/----------------------------------------------------------------*/
#include "ecRCC2.h"
#include "ecGPIO2.h"
//#include "ecSysTick.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

volatile uint32_t msTicks = 0;
volatile uint32_t curTicks;
volatile uint32_t msDelay=1000;


void setup(void)
{
    RCC_PLL_init();                         // System Clock = 84MHz
    GPIO_init(LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, PULL_UP);
    GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
}

void SysTick_Handler(){
    msTicks++;
}

int led_state = 1;
void LED_toggle() {
    GPIO_write(LED_PIN, !led_state);
    led_state = !led_state;
}

int main(void) {
    // System CLOCK, GPIO Initialiization ----------------------------------------
    setup();

    // SysTick Initialiization ------------------------------------------------------
    //  SysTick Control and Status Register
    SysTick->CTRL = 0;											// Disable SysTick IRQ and SysTick Counter

    // Select processor clock
    // 1 = processor clock;  0 = external clock
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    // uint32_t MCU_CLK=EC_SYSTEM_CLK
    // SysTick Reload Value Register
    SysTick->LOAD = MCU_CLK_PLL/1000-1;						// 1ms

    // SysTick Current Value Register
    SysTick->VAL = 0;

    // Enables SysTick exception request
    // 1 = counting down to zero asserts the SysTick exception request
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    // Enable SysTick IRQ and SysTick Timer
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
    NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC

    msTicks = 0;
    while(1) {
        curTicks = msTicks;
        while ((msTicks - curTicks) < 1000);
        msTicks = 0;
        LED_toggle();
    }
}


