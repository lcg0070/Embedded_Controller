#include "ecSTM32F4.h"
// #include "ecUART.h"

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
uint8_t PC_string[]="Loop:\r\n";

void setup(void) {
    RCC_PLL_init();
    SysTick_init();

    // USART2: USB serial init for PC communication
    UART2_init();
    UART2_baud(BAUD_9600);

    // USART1: Bluetooth serial init for HC-06
    UART1_init();
    UART1_baud(BAUD_9600);
}

int main(void) {
    setup();
    printf("MCU Initialized\r\n");

    while(1) {
        // Transmit to PC through USART2 every 2 seconds
        USART2_write(PC_string, sizeof(PC_string) - 1);
        delay_ms(2000);

        // Optionally, add code to handle received Bluetooth data if needed
        if (BT_Data != 0) {
            printf("Received from Bluetooth: %c\r\n", BT_Data);
            BT_Data = 0; // Clear BT_Data after handling
        }
    }
}

// USART2 IRQ Handler for PC Communication
void USART2_IRQHandler() {
    if (is_USART2_RXNE()) {
        PC_Data = USART2_read();         // Read data from PC
        USART2_write(&PC_Data, 1);       // Echo back to PC
    }
}

// USART1 IRQ Handler for Bluetooth (HC-06) Communication
void USART1_IRQHandler() {

    if (is_USART1_RXNE()) {
        BT_Data = USART1_read();         // Read data from Bluetooth
        printf("Received from Bluetooth: %c\r\n", BT_Data);  // Print to PC via USART2
    }
}
