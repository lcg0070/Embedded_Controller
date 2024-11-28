#include <string.h>
#include "ecSTM32F4.h"
#include "smartFarm_Hani.h"

static volatile uint8_t BT_Response[100];
static volatile uint8_t BT_Data = 0;
static volatile int Response_Index = 0;


int on_state = 0;
int main() {
    communication_send_setup();

    while (1) {
        on_state = 0;
        if (GPIO_read(COMMUNICATION_SEND_PINA) == HIGH) {on_state = 1;}
        if (GPIO_read(COMMUNICATION_SEND_PINB) == HIGH) {on_state = 2;}
        if (GPIO_read(COMMUNICATION_SEND_PINC) == HIGH) {on_state = 3;}
        if (GPIO_read(COMMUNICATION_SEND_PIND) == HIGH) {on_state = 4;}


        if (on_state) {
            // USART1_write((uint8_t *)"1", 1); // Send "1" to Slave
            GPIO_write(LED_PIN, HIGH);
        }else{
            GPIO_write(LED_PIN, LOW);
        }

    }
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