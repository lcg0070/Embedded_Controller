#include <string.h>
#include "ecSTM32F4.h"
#include "smartFarm_Hani.h"

static volatile uint8_t BT_Response[100];
static volatile uint8_t BT_Data = 0;
static volatile int Response_Index = 0;

uint8_t button_state_history = 0;
uint8_t button_state_current = 0;

int main() {
    communication_send_setup();
    while (1) {
        process_button_states(&button_state_current, &button_state_history);
    }
    return 0;
}


// USART1 IRQ Handler for Bluetooth Communication
void USART1_IRQHandler() {
    if (is_USART1_RXNE()) {
        BT_Data = USART1_read(); // Read data from Bluetooth
        if (Response_Index < sizeof(BT_Response) - 1) {
            BT_Response[Response_Index++] = BT_Data;
        }else {
            Response_Index = 0;
        }
    }
}
