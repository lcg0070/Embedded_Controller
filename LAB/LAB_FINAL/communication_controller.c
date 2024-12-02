#include <string.h>
#include "ecSTM32F4.h"
#include "smartFarm_Hani.h"


uint8_t button_state_history = 0;
uint8_t button_state_current = 0;

int main() {
    communication_send_setup();
    while (1) {
        process_button_states(&button_state_current, &button_state_history);
        delay_ms(100);
    }
    return 0;
}

