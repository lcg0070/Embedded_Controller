/*----------------------------------------------------------------\
Author           : Lee ChanKeun
Created          : 10-15-2024
Modified         : 10-15-2024
Language/ver     : C in CLION with platformio

Description      : MID_TEST1
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "math.h"
#include "ecPinNames.h"
#include "ecGPIO2.h"
#include "ecSysTick2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"
#include "ecEXTI2.h"

#define PWM_PIN         PA_0
#define DIRECTION_PIN   PC_2
#define IR_PIN          PB_1


void setup(void);

void set_OFF(void);

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

    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);

    // Button pin
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_otype(BUTTON_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(BUTTON_PIN, PULL_DOWN);
    EXTI_init(BUTTON_PIN, FALL,2);

    // IR Sensor
    GPIO_init(IR_PIN, INPUT);
    GPIO_otype(IR_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(IR_PIN, PULL_DOWN);
    EXTI_init(IR_PIN, RISE, 1);

    // PWM
    PWM_init(PWM_PIN, M_SEC, 1);
    PWM_period(PWM_PIN, M_SEC ,1);

    // DIRECTION_PIN
    GPIO_init(DIRECTION_PIN, OUTPUT);
    GPIO_otype(DIRECTION_PIN, OUTPUT_PUSH_PULL);
    GPIO_ospeed(DIRECTION_PIN, HIGH_SPEED);

    TIM_UI_init(TIM3, M_SEC, 500);
    TIM_UI_enable(TIM3);

    PWM_duty(PWM_PIN, 0);
}


// ================================================
// change directions
// ================================================

// tim INTERRUPT
uint32_t tim_count          = 1;
uint32_t tim_count_speed    = 0;

// 7seg-display
uint32_t seven_count        = 0;

float targetPWM = 0.f;  // pwm for motor input

float DIR = 0.f;
float duty;

void TIM3_IRQHandler(void){
    if(is_UIF(TIM3)){
        // stop when speed == 0
        if(tim_count_speed == 0) {
            set_OFF();
            return;
        }

        // timer condition
        if(tim_count > tim_count_speed) {

            // 7-segment_display
            sevensegment_display(seven_count);
            if(DIR == 1.f) {
                if(seven_count == 0) {
                    seven_count = 9;
                }else {
                    seven_count--;
                }
            }else seven_count++;
            if(seven_count > 9) seven_count = 0;

            // PWM motor
            // motor speed according to timer speed
            if(tim_count_speed == 2) targetPWM = 0.75;
            else if(tim_count_speed == 1) targetPWM = 0.25;

            targetPWM = fabs(1.f - targetPWM);
            duty = fabs(DIR - targetPWM);
            PWM_duty(PWM_PIN, duty);
            tim_count = 1;
        }

        tim_count++;
        clear_UIF(TIM3);
    }
}


// BUTTON Interrupt
void EXTI15_10_IRQHandler(void) {
    //check pending
    if(is_pending_EXTI(BUTTON_PIN) ) {

        //debouncing
        for(int i=0; i<30000; i++){}

        // time duty change of 7-segment
        if(tim_count_speed == 2) {
            tim_count_speed = 1;

        }else if(tim_count_speed == 1) {
            // change dir
            tim_count_speed = 0;

            // direction change of the motor
            if(DIR == 0.f) DIR = 1.f;
            else DIR = 0.f;

            // error exception of number
            if(DIR == 1.f) {
                if(seven_count == 0) {
                    seven_count = 9;
                }else {
                    seven_count--;
                }
            }else seven_count++;
            if(seven_count > 9) seven_count = 0;

            targetPWM = 0;

        }else {
            tim_count_speed = 2;
        }

        // PWM motor
        GPIO_write(DIRECTION_PIN, (int)DIR);
        duty = fabs(DIR - targetPWM);
        PWM_duty(PWM_PIN, duty);

        //clear pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}


// IR Interrupt
void EXTI1_IRQHandler(void) {
    //check pending
    if(is_pending_EXTI(IR_PIN) ) {


        DIR = 0.f;
        tim_count = 0;
        seven_count = 0;
        targetPWM = 0;


        while(GPIO_read(IR_PIN) == 0) {
            set_OFF();
        }

        // reset state
        tim_count_speed = 0;
        targetPWM = 0.;
        duty = fabs(DIR - targetPWM);
        PWM_duty(PWM_PIN, duty);


        //clear pending
        clear_pending_EXTI(IR_PIN);
    }
}


void set_OFF() {

    GPIO_write(DIRECTION_PIN, (int)DIR);
    // PA_7, PB_6, PC_7, PA_9
    GPIO_write(PA_7, HIGH);
    GPIO_write(PB_6, HIGH);
    GPIO_write(PC_7, HIGH);
    GPIO_write(PA_9, HIGH);

    PWM_duty(PWM_PIN, DIR);
}