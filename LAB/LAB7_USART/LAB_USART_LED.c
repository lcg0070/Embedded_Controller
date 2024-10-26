#include "ecSTM32F4.h"
// #include "ecUART.h"



static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
uint8_t PC_string[]="Loop:\r\n";

void setup(void){
    RCC_PLL_init();
    SysTick_init();

    // USART2: USB serial init
    UART2_init();
    UART2_baud(BAUD_9600);

    // USART1: BT serial init
    UART1_init();
    UART1_baud(BAUD_9600);
}

int main(void){
    setup();
    printf("MCU Initialized\r\n");
    while(1){
        // USART Receive: Use Interrupt only
        // USART Transmit:  Interrupt or Polling
        USART2_write(PC_string, 7);
        delay_ms(2000);
    }
}

void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
    if(is_USART2_RXNE()){
        PC_Data = USART2_read();		// RX from UART2 (PC)
        USART2_write(&PC_Data,1);		// TX to USART2	 (PC)	 Echo of keyboard typing
    }
}


void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();		// RX from UART1 (BT)
        printf("RX: %c \r\n",BT_Data); // TX to USART2(PC)
    }
}