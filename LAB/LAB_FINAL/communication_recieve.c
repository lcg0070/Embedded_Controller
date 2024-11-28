#include <string.h>
#include "ecSTM32F4.h"


static volatile uint8_t BT_Data = 0;
#define LED_PIN PA_5 // LED Pin

void send_AT_command(const char *cmd, uint32_t delay) {
    printf("Sending AT Command: %s\r\n", cmd);
    USART1_write((uint8_t *)cmd, strlen(cmd));
    delay_ms(delay); // Wait for response
}

// USART1 IRQ Handler for Bluetooth Communication
void USART1_IRQHandler() {
    if (is_USART1_RXNE()) {
        BT_Data = USART1_read(); // Read data from Bluetooth
    }
}


void setup() {
    RCC_PLL_init();
    SysTick_init();

    // USART2: Debug output to PC
    UART2_init();
    UART2_baud(BAUD_9600);

    // USART1: Bluetooth (HC-06)
    UART1_init();
    UART1_baud(BAUD_9600);

    // Initialize LED
    GPIO_init(LED_PIN, OUTPUT);
}

int main() {
    setup();

    while (1) {
        if (BT_Data == '1') { // If "1" received from Master
            GPIO_write(LED_PIN, HIGH);
            delay_ms(1000); // LED stays on for 1 second
            GPIO_write(LED_PIN, LOW);
            BT_Data = 0;    // Clear data
            printf("LED ON: Received '1'\r\n");
        }
    }
}
