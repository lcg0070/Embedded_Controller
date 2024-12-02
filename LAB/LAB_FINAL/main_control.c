

#include <string.h>
#include "ecSTM32F4.h"
#include "smartFarm_Hani.h"

// Flags
// 0: AUTO, MANUAL            ON,OFF
// 1: WATER_MOTOR             ON,OFF
// 2: Hydroponic_Nutrients    ON,OFF
// 3: LED, MAIN_PUMP          ON,OFF
uint8_t flags[4] = {0, 0, 0, 0};

float ph_level = 0;
volatile uint32_t msTicks = 0;  // Millisecond counter
char currentTime[6] = "12:34";

void sendDataUART2(float pH, char* time);

int main() {
    main_setup();
    while (1) {
        process_farm(flags, msTicks);
        sendDataUART2(ph_level, currentTime);
    }
}


void TIM3_IRQHandler(void){
    if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
        msTicks++;              // Increment millisecond counter
        clear_UIF(TIM3); 		// Clear UI flag by writing 0
    }
}


// USART1 IRQ Handler for Bluetooth Communication
void USART1_IRQHandler() {
    blutooth_data2flag(flags);
}


// USART2 communication to arduino
void sendDataUART2(float pH, char* time) {
    uint8_t buffer[32];
    sprintf(buffer, "%.2f,%s", pH, time); // "pH값,시간" 형식
    USART2_write((uint8_t *)buffer, sizeof(buffer));
}

void ADC_IRQHandler(void){
    ph_level = cal_ph();
}
