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

void slave_setup_AT_mode() {
    printf("Entering AT Mode\r\n");
    send_AT_command("AT", 1000);             // Test AT mode
    send_AT_command("AT+ROLE=S", 1000);      // Set as Slave
    send_AT_command("AT+CMODE=0", 1000);     // Pair with specific address
    send_AT_command("AT+BIND=1234,56789AB", 1000); // Bind to Master
    send_AT_command("AT+RESET", 2000);       // Apply and reboot
    printf("Slave AT setup complete\r\n");


    UART1_baud(BAUD_9600);                   // AT Mode default baud rate
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
    slave_setup_AT_mode(); // Configure HC-06 as Slave

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
