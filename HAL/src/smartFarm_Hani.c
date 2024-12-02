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

void UART2_setup() {
    UART2_init();
    UART2_baud(BAUD_9600);
}



// =======================================
// Communication Send
// =======================================


void communication_send_setup() {
    RCC_PLL_init();
    SysTick_init();

    LED_setup();
    communication_send_init();
    UART1_setup();
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

void process_button_states(uint8_t *button_state_current, uint8_t *button_state_history) {
    *button_state_current = 0;
    for (int i = 0; i<4; i++) {
        *button_state_current |= (GPIO_read(pin[i]) == HIGH) << i;
    }
    uint8_t changed_buttons = *button_state_current ^ *button_state_history;
    *button_state_history = *button_state_current;

    if (changed_buttons) {
        USART1_write(button_state_current, sizeof(*button_state_current));
        if (*button_state_current) {
            GPIO_write(LED_PIN, HIGH);
        } else {
            GPIO_write(LED_PIN, LOW);
        }
    }
}


// =======================================
// Communication Receive
// =======================================
void main_setup() {
    RCC_PLL_init();
    SysTick_init();
    communication_recieve_setup();
}

void communication_recieve_setup() {
    LED_setup();
    UART2_setup();
    UART1_setup();

}

static volatile uint8_t BT_Data = 0;
void blutooth_data2flag(uint8_t flags[]) {
    if (is_USART1_RXNE()) {
        BT_Data = USART1_read();
        if(BT_Data == 0) return;
        for (int i = 0; i < 4; i++) {
            if ( BT_Data >> i & 0b1) {
                flags[i] = !flags[i];
            }
        }
    }
}