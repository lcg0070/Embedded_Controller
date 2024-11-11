// /*----------------------------------------------------------------\
// Author           : Lee ChanKeun
// Created          : 11-05-2024
// Modified         : 11-05-2024
// Language/ver     : C in CLION with platformio
//
// Description      : LAB_RC_CAR
// /----------------------------------------------------------------*/

#include <math.h>

#include "stm32f411xe.h"
#include "RC_CAR.h"
#include "ecSTM32F4.h"


uint32_t IR_Value[2] = { 0, 0 };
float ultrasonic_distance = 0.0f;

uint8_t mode = MANUAL;
uint8_t btData = 0;
uint8_t buffer[20] ={0,};

void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1) {
        printf("%lu IR1\t %lu IR2\t distance = %f\t mode = %d \r\n", IR_Value[0], IR_Value[1], ultrasonic_distance, mode);
        motor_control(IR_Value, ultrasonic_distance, btData, mode);
    }
}

// Initialiization
void setup(void){
    RCC_PLL_init();                 // System Clock = 84MHz
    SysTick_init();			         // SysTick Init

    // communication
    UART2_init();

    // each setup
    UART1_setup();
    DC_setup();
    ultrasonic_setup();
    LED_setup();
    ADC_setup();                   // priority 2
}


// =======================================
// interrupt
// =======================================

// LED control
void TIM3_IRQHandler(void){
    led_control(mode);
}


// ultra sonic interrupt
void TIM4_IRQHandler(void){
    ultrasonic_distance = get_distance(ultrasonic_control());
    if (ultrasonic_distance > 1000) ultrasonic_distance = ULTRASONIC_THRESHOLD + 1;
    if(ultrasonic_distance < 0) ultrasonic_distance = ULTRASONIC_THRESHOLD + 1;             // overflow
}

// IR sensor interrupt
void ADC_IRQHandler(void){
    IR_control(IR_Value);
}


void USART1_IRQHandler() {  // USART1 interrupt handler
    if (is_USART_RXNE(USART1)) {
        int arrow_flag = 0;
        btData = USART_read(USART1);

        if (btData == 0x1B) {  // Esc -> special character
            btData = USART_read(USART1);
            if (btData == 0x5B) {  // '['
                // check arrow
                btData = USART_read(USART1);
                arrow_flag = 1;
            }
        }
        Direction(btData, arrow_flag);             // Execute direction control based on btData
        Direction_display(btData, arrow_flag);     // Display current direction command
        if( btData == MODE_CHANGE1 || btData == MODE_CHANGE2) {
            mode = mode_toggle(mode);
        }
    }
}



