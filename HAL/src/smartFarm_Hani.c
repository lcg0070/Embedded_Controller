//
// Created by 이찬근 on 2024. 11. 19..
//

#include "smartFarm_Hani.h"
#include "ecSTM32F4.h"




// =======================================
// setup
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

