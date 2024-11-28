#include <string.h>
#include "ecSTM32F4.h"
#include "smartFarm_Hani.h"

static volatile uint8_t BT_Data = 0;

// Flags
// 0: AUTO, MANUAL
// 1: WATER_MOTOR             ON,OFF
// 2: Hydroponic_Nutrients    ON,OFF
// 3: LED, MAIN_PUMP          ON,OFF

uint8_t flags[4] = {0, 0, 0, 0};

int main() {
    communication_recieve_setup();

    while (1) {

    }
}



// main loop
void TIM3_IRQHandler() {

}


// USART1 IRQ Handler for Bluetooth Communication
void USART1_IRQHandler() {
    if (is_USART1_RXNE()) {
        BT_Data = USART1_read(); // Read data from Bluetooth

        for (int i = 0; i < 4; i++) {
            if ( (BT_Data >> i) & 0b1) {
                flags[i] = !flags[i];
            }
        }
    }
}