// /*----------------------------------------------------------------\
// Author           : Lee ChanKeun
// Created          : 11-19-2024
// Modified         : 11-19-2024
// Language/ver     : C in CLION with platformio
//
// Description      : LAB_Final_Header
// /----------------------------------------------------------------*/

#ifndef SMARTFARM_HANI_H
#define SMARTFARM_HANI_H

#include "stm32f411xe.h"

// =======================================
// Communication Send
// =======================================
#define COMMUNICATION_SEND_PINA PA_6
#define COMMUNICATION_SEND_PINB PB_6
#define COMMUNICATION_SEND_PINC PC_7
#define COMMUNICATION_SEND_PIND PB_10




// =======================================
// Common
// =======================================

void UART1_setup();
void LED_setup();


// =======================================
// Communication Send
// =======================================

void communication_send_setup();
void communication_send_init();
void process_button_states(uint8_t *button_state_current, uint8_t *button_state_history);



// =======================================
// Communication Receive
// =======================================
void communication_recieve_setup();
void communication_recieve_init();


#endif //SMARTFARM_HANI_H
