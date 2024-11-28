//
// Created by 이찬근 on 2024. 11. 19..
//

#include "smartFarm_Hani.h"
#include "ecSTM32F4.h"



// =======================================
// Common
// =======================================

void LED_setup() {
    // LED pin
    GPIO_init(LED_PIN, OUTPUT);
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, PULL_UP);
    GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
}


void UART1_setup() {
    UART1_init();
    UART1_baud(BAUD_9600);
}




// =======================================
// Communication Send
// =======================================


void communication_send_setup() {
    RCC_PLL_init();
    SysTick_init();

    UART1_setup();
    communication_send_init();
}


#define PIN_INDEX (int)4
PinName_t pin[PIN_INDEX];

void communication_send_init() {
    pin[0]  =   COMMUNICATION_SEND_PINA;
    pin[1]  =   COMMUNICATION_SEND_PINB;
    pin[2]  =   COMMUNICATION_SEND_PINC;
    pin[3]  =   COMMUNICATION_SEND_PIND;

    for(int i = 0; i < PIN_INDEX; i++) {
        GPIO_init(pin[i], INPUT);
        GPIO_pupd(pin[i], PULL_DOWN);
    }
    GPIO_init(LED_PIN, OUTPUT);
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, PULL_DOWN);
    GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
}