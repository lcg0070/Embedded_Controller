// /*----------------------------------------------------------------\
// Author           : Lee ChanKeun
// Created          : 10-08-2024
// Modified         : 10-08-2024
// Language/ver     : C in CLION with platformio
//
// Description      : LAB_TimerPWM
// /----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO2.h"
#include "ecSysTick2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"   // ecPWM2.h


// Definition Button Pin & PWM Port, Pin
#define PWM_PIN PA_5
void setup(void);

uint32_t count = 0;

int led_state = 1;
void LED_toggle() {
    GPIO_write(PWM_PIN, !led_state);
    led_state = !led_state;
}


int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while(1){
    }
}

// Initialiization
void setup(void) {
    RCC_PLL_init();
    SysTick_init();
    TIM_UI_init(TIM3, M_SEC, 50);
    TIM_UI_enable(TIM3);

    // PWM of 20 msec:  TIM2_CH1 (PA_5 AFmode)
    GPIO_init(PWM_PIN, AF);
    // PWM_init(PWM_PIN, M_SEC, 1);
    PWM_init(PWM_PIN, M_SEC, 1);
    PWM_period(PWM_PIN, M_SEC ,20);
}

// 2.5ms~0.5ms(180~0)
// 2/18 -> per 10 degree
// duty = (0.5+(1/9)*i)/20.
int i=0;
int timer_flag = 0;
void TIM3_IRQHandler(void){
    if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
        count++;
        if(count > 9) {
            PWM_duty(PWM_PIN, (float)(0.5+(1./9.)*i)/20.);
            count=0;
            if(timer_flag) i--;
            else i++;;
            if(i <0 || i>18) timer_flag = !timer_flag;
        }
        clear_UIF(TIM3); 		// Clear UI flag by writing 0
    }
}


