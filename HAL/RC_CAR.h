////
//// Created by 이찬근 on 2024. 11. 5..
////
//
//#ifndef RC_CAR_H
//#define RC_CAR_H
//
//
//#include "ecSTM32F4.h"
////=============================================================================
//// pin config
////=============================================================================
//
////Ultrasonic parameter//
//#define TRIG PA_8               //TIM1
//#define ECHO PB_6               //TIM4
//
////IR parameter//                TIM5
//#define IR_PIN_1 PB_0
//#define IR_PIN_2 PB_1
//
////Motor parameter//             //TIM2
//#define PWM_PIN1         PA_0
//#define PWM_PIN2         PA_6
//#define DIRECTION_PIN1   PC_2
//#define DIRECTION_PIN2   PC_3
//
////Blutooth parameter//
//#define RX_PIN           PA_9
//#define TX_PIN           PA_10
//
//
////=============================================================================
//// motor speed
////=============================================================================
//#define    HIGH_MOTOR_SPEED         (1.0f)
//#define    MIDDLE_MOTOR_SPEED       (0.6f)
//#define    LOW_MOTOR_SPEED          (0.3f)
//#define    STOP_MOTOR_SPEED         (0.0f)
//
//#define    DUTY_CYCLE_STEP          (0.33f)
//
//#define    FORWARD_DIR            (0.f)
//#define    BACKWARD_DIR           (1.f)
////=============================================================================
//// mode
////=============================================================================
//#define MANUAL    1
//#define AUTO      2
//
//// manual define
//#define UP              (0x41)
//#define DOWN            (0x42)
//#define RIGHT           (0x43)
//#define LEFT            (0x44)
//#define STOP            'S'
//#define FORWARD         'F'
//#define BACKWARD        'B'
//#define MODE_CHANGE1    'M'
//#define MODE_CHANGE2    'm'
//
//#define ASCII_OFFSET    0x20;
//
//// ultrasonic sensor threshold
//#define ULTRASONIC_THRESHOLD   5.f       //[cm]
//
//// IR sensor threshold
//#define IR_THRESHOLD          2500.f       //[-]
//
//
//
//// =======================================
//// setup
//// =======================================
//void ADC_setup();
//void LED_setup();
//void DC_setup();
//void ultrasonic_setup();
//void UART1_setup();
//
//
//
//// =======================================
//// DEFAULT
//// =======================================
//
//// motor output
//void motor_control(uint32_t IR_value[], float ultrasonic_distance, uint8_t bt_data ,uint8_t mode);
//void motor_auto(uint32_t IR_value[], float ultrasonic_distance, uint8_t initial_flag);
//void motor_manual(uint8_t bt_data, uint8_t initial_flag);
//
//void led_control(uint8_t mode);
//uint8_t mode_toggle(uint8_t mode);
//
//// =======================================
//// AUTO MODE
//// =======================================
//void IR_control(uint32_t IR_value[]);
//float ultrasonic_control();
//float get_distance(float timeinterval);
//
//
//// =======================================
//// MANUAL MODE
//// =======================================
//void set_car_dir(uint8_t dir);
//void set_car_speed(float speed_left, float speed_right, float direction);
//void Direction(uint8_t direction, uint8_t arrow_flag);
//void Direction_display(uint8_t direction, uint8_t arrow_flag);
//
//
//#endif //RC_CAR_H
