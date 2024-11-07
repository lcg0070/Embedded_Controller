//
// Created by 이찬근 on 2024. 11. 5..
//

#ifndef RC_CAR_H
#define RC_CAR_H


#include "ecSTM32F4.h"
//=============================================================================
// pin config
//=============================================================================

//Ultrasonic parameter//
#define TRIG PA_6               //PA_8
#define ECHO PB_6

//IR parameter//
#define IR_PIN_1 PB_0
#define IR_PIN_2 PB_1

//Motor parameter//
#define PWM_PIN1         PA_0
#define PWM_PIN2         PA_1            // PA_3
#define DIRECTION_PIN1   PC_2
#define DIRECTION_PIN2   PC_3

//Blutooth parameter//
#define RX_PIN           PA_9
#define TX_PIN           PA_10


//=============================================================================
// motor speed
//=============================================================================
#define    HIGH_MOTOR_SPEED         (1.0f)
#define    MIDDLE_MOTOR_SPEED       (0.6f)
#define    LOW_MOTOR_SPEED          (0.3f)


//=============================================================================
// mode
//=============================================================================
#define MANUAL    1
#define AUTO      2

// manual key
#define UP       'W'
#define DOWN     'X'
#define RIGHT    'D'
#define LEFT     'A'
#define STOP     'S'
#define FORWARD  'F'
#define BACKWARD 'B'
#define MODE     'M'



// ultrasonic sensor threshold
#define ULTRASONIC_THRESHOLD   5.f       //[cm]

// IR sensor threshold
#define IR_THRESHOLD          500.f       //[-]



// =======================================
// setup
// =======================================
void ADC_setup();
void DC_setup();
void ultrasonic_setup();
void bluetooth_setup();



// =======================================
// interrupt
// =======================================
void auto_control(uint32_t IR_value[], float ultrasonic_distance);
void manual_control();

void IR_control(uint32_t IR_value[]);
float ultrasonic_control();
float get_distance(float timeinterval);

#endif //RC_CAR_H
