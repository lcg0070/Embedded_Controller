#include <string.h>
#include "ecSTM32F4.h"

static volatile uint8_t BT_Response[100];
static volatile uint8_t BT_Data = 0;
static volatile int Response_Index = 0;
#define LED_PIN PA_5  // LED Pin
#define BTN_PIN PC_13 // Button Pin

void send_AT_command(const char *cmd, uint32_t delay) {
    printf("Sending AT Command: %s\r\n", cmd);
    USART1_write((uint8_t *)cmd, strlen(cmd));  // 명령어 전송
    delay_ms(delay);                            // 응답 대기

}

// USART1 IRQ Handler for Bluetooth Communication
void USART1_IRQHandler() {
    if (is_USART1_RXNE()) {
        BT_Data = USART1_read(); // Read data from Bluetooth
        if (Response_Index < sizeof(BT_Response) - 1) {
            BT_Response[Response_Index++] = BT_Data;
        }
    }
}

void master_setup_AT_mode() {
    printf("Entering AT Mode\r\n");
    send_AT_command("AT", 1000);             // Test AT mode
    send_AT_command("AT+ROLE=M", 1000);      // Set as Master
    send_AT_command("AT+CMODE=0", 1000);     // Pair with specific address
    send_AT_command("AT+BIND=1234,56789AB", 1000); // Bind to Slave
    send_AT_command("AT+RESET", 2000);       // Apply and reboot
    printf("Master AT setup complete\r\n");

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

    // Initialize LED and Button
    GPIO_init(LED_PIN, OUTPUT);
    GPIO_init(BTN_PIN, INPUT);
}

int main() {
    setup();
    master_setup_AT_mode(); // Configure HC-06 as Master

    while (1) {
        if (GPIO_read(BTN_PIN) == 0) { // Button pressed
            GPIO_write(LED_PIN, HIGH);
            USART1_write((uint8_t *)"1", 1); // Send "1" to Slave
            delay_ms(100);
            printf("LED ON: send '1'\r\n");
        } else {
            GPIO_write(LED_PIN, LOW);
        }
    }
}
