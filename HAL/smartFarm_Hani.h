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
#define COMMUNICATION_SEND_PINA     PA_6
#define COMMUNICATION_SEND_PINB     PB_6
#define COMMUNICATION_SEND_PINC     PC_7
#define COMMUNICATION_SEND_PIND     PB_10


// =======================================
// Main Control
// =======================================

// Timer
#define TIMER_DUTY_MSEC            (uint32_t)(500)           // 500ms
#define TIMER_TICK_COUNT_THRESHOLD (uint32_t)(7200)          // 500[ms] * 7200 -> 1 [hour]


// PH_SENSOR
#define PH_SENSOR_PIN               PA_0
#define PH_OFFSET                   (float)(0.)
#define PH_VOLTAGE_SCALE_FACTOR     (float)(3.5 / 4096.)


// motor
#define MAIN_PUMP_PIN               PA_1
#define WATER_SUPPLY_PIN            PA_1
#define NUTRIENT_SUPPLY_PIN         PA_1

// LED
#define FARM_LED_PIN                PA_1



// =======================================
// Common
// =======================================
void LED_setup();
void UART1_setup();
void UART2_setup();


// =======================================
// Communication Send
// =======================================

void communication_send_setup();
void communication_send_init();
void process_button_states(uint8_t *button_state_current, uint8_t *button_state_history);



// =======================================
// Main Control
// =======================================
void main_setup();

//==============
// communication
//==============
void communication_recieve_setup();
void communication_recieve_init();
void blutooth_data2flag(uint8_t flags[]);

//============
// ph sensor
//============
void ph_sensor_setup();
float cal_ph();
float ph_value2level(float ph_value);

//============
// flag control
//============
// 0: AUTO, MANUAL            ON,OFF
// 1: WATER_MOTOR             ON,OFF
// 2: Hydroponic_Nutrients    ON,OFF
// 3: LED, MAIN_PUMP          ON,OFF

void process_farm(uint8_t flags[], uint32_t current_time);
void handle_mode_transition(uint8_t current_mode, uint32_t *last_toggle_time);
void control_auto_mode();
void control_manual_mode();

#endif //SMARTFARM_HANI_H
