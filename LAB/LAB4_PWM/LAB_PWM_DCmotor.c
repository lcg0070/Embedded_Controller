/*----------------------------------------------------------------\
Author           : Lee ChanKeun
Created          : 10-08-2024
Modified         : 10-09-2024
Language/ver     : C in CLION with platformio

Description      : LAB_PWM_DCmotor
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "math.h"
// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO2.h"
#include "ecSysTick2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"   // ecPWM2.h
#include "ecEXTI2.h"

// Definition Button Pin & PWM Port, Pin
#define PWM_PIN         PA_0
#define DIRECTION_PIN   PC_2

void setup(void);

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

    TIM_UI_init(TIM3, M_SEC, 500);
    TIM_UI_enable(TIM3);

    // PWM
    PWM_init(PWM_PIN, M_SEC, 1);
    PWM_period(PWM_PIN, M_SEC ,1);

    // Button pin
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_otype(BUTTON_PIN, OUTPUT_PUSH_PULL);

    EXTI_init(BUTTON_PIN, FALL,1);

    GPIO_init(DIRECTION_PIN, OUTPUT);
    GPIO_otype(DIRECTION_PIN, OUTPUT_PUSH_PULL);
    GPIO_ospeed(DIRECTION_PIN, HIGH_SPEED);

}

// 2.5ms~0.5ms(180~0)
// 2/18 -> per 10 degree
// duty = (0.5+(1/9)*i)/20
// PWM INTERRUPT
int run_flag = 1;
uint32_t count = 0;


float targetPWM = 0.25f;  // pwm for motor input
float DIR = 0.f;
float duty; // duty with consideration of DIR=1 or 0

void TIM3_IRQHandler(void){
    if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
        if(count > 3) {
            targetPWM = fabs(1.f - targetPWM);
            duty = fabs(DIR - targetPWM);
            PWM_duty(PWM_PIN, duty);
            count = 0;
        }
        count++;
        clear_UIF(TIM3); 		// Clear UI flag by writing 0
    }
}

// BUTTON Interrupt
void EXTI15_10_IRQHandler(void) {
    //check pending
    if(is_pending_EXTI(BUTTON_PIN) ) {
        //debouncing
        for(int i=0; i<30000; i++){}

        if(DIR == 0.f) DIR = 1.f;
        else DIR = 0.f;
        GPIO_write(DIRECTION_PIN, (int)DIR);
        duty = fabs(DIR - targetPWM);
        PWM_duty(PWM_PIN, duty);
        //clear pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}