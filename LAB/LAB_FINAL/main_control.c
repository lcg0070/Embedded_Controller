

#include <string.h>
#include "ecSTM32F4.h"
#include "smartFarm_Hani.h"

// Flags
// 0: AUTO, MANUAL            ON,OFF
// 1: WATER_MOTOR             ON,OFF
// 2: Hydroponic_Nutrients    ON,OFF
// 3: LED, MAIN_PUMP          ON,OFF
uint8_t flags[4] = {0, 0, 0, 0};

float pHValue = 7.25;
char currentTime[6] = "12:34";

void sendDataUART2(float pH, char* time);

int main() {
    main_setup();
    while (1) {
        pHValue = 10.0;
        for(int i = 0; i < 4; i++) {
            if(flags[i] == HIGH) {
                GPIO_write(LED_PIN, HIGH);
                pHValue = 0.0;
                break;
            }
        }
        delay_ms(100);
        GPIO_write(LED_PIN, LOW);

        sendDataUART2(pHValue, currentTime);
    }
}


// USART1 IRQ Handler for Bluetooth Communication
void USART1_IRQHandler() {
    blutooth_data2flag(flags);
}


// USART2를 통해 pH 값과 시간 송신
void sendDataUART2(float pH, char* time) {
    uint8_t buffer[32];
    sprintf(buffer, "%.2f,%s", pH, time); // "pH값,시간" 형식
    USART2_write((uint8_t *)buffer, sizeof(buffer));
}

