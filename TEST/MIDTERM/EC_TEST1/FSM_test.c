/*----------------------------------------------------------------\
Author           : Lee ChanKeun
Created          : 10-15-2024
Modified         : 10-15-2024
Language/ver     : C in CLION with platformio

Description      : MID_TEST1
/----------------------------------------------------------------*/

#include <stdlib.h>

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
#define STATE_INDEX     6

void setup(void);
void set_off();
void motor_off();
void sevensegment_off();
void init_states(void);
void set_state();


// state
typedef struct {
    uint8_t PWM_DIR;
    float   PWM_OUT;
    uint8_t GPIO_STEP;
}fsm_state;

uint8_t PWM_DIRS[6] = {0, 0, 0, 0, 1, 1};
float   PWM_OUTS[3] = {0, 0.25f, 0.75f};
uint8_t GPIO_STEPS[3] = {1, 3, 2};

fsm_state states[STATE_INDEX];

fsm_state current_state;
int state_idx = 0;



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

    init_states();
    current_state = states[0];
}





// ================================================
// flags
// ================================================

int button_flag = 0;
int ir_flag = 0;

void TIM3_IRQHandler(void) {
    if(is_UIF(TIM3)){

        if(ir_flag) {
            state_idx = 0;
            ir_flag = 0;

        }else {
            if(button_flag) {
                // clear flag
                button_flag = 0;

                state_idx++;

                if(state_idx > STATE_INDEX-1) state_idx = 0;
            }
        }
        current_state = states[state_idx];
        set_state();
        clear_UIF(TIM3);
    }
}


// BUTTON Interrupt
void EXTI15_10_IRQHandler(void){
    //check pending
    if(is_pending_EXTI(BUTTON_PIN) ) {
        button_flag = 1;
        //clear pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}


// IR Interrupt
void EXTI1_IRQHandler(void){
    //check pending
    if(is_pending_EXTI(IR_PIN) ) {

        while(GPIO_read(IR_PIN) == 0) {
            set_off();
        }
        ir_flag = 1;
        //clear pending
        clear_pending_EXTI(IR_PIN);
    }
}


void init_states(void) {
    for(int i = 0; i<STATE_INDEX; i++) {
        states[i].PWM_DIR = PWM_DIRS[i];
        states[i].PWM_OUT = PWM_OUTS[i%3];
        states[i].GPIO_STEP = GPIO_STEPS[i%3];
    }
}

void set_state(){
    // motor

    float DIR = current_state.PWM_DIR;
    float PWM_OUT = current_state.PWM_OUT;
    float duty = fabs(DIR - PWM_OUT);
    GPIO_write(DIRECTION_PIN, (int)DIR);
    PWM_duty(PWM_PIN, duty);

}

void set_off(void){
    // PWM_PIN
    motor_off();

    // PA_7, PB_6, PC_7, PA_9
    sevensegment_off();
}


void motor_off() {

    GPIO_write(DIRECTION_PIN, (int)1);
    PWM_duty(PWM_PIN, 1);
}

void sevensegment_off() {
    GPIO_write(PA_7, HIGH);
    GPIO_write(PB_6, HIGH);
    GPIO_write(PC_7, HIGH);
    GPIO_write(PA_9, HIGH);
}