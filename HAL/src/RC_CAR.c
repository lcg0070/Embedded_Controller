//
// Created by 이찬근 on 2024. 11. 5..
//

#include "RC_CAR.h"
#include "ecSTM32F4.h"
#include "math.h"

// //IR parameter//
uint32_t value1, value2;
PinName_t seqCHn[2] = {IR_PIN_1, IR_PIN_2};

int flag = 0;


//Ultrasonic parameter//
uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;


//Motor parameter//
int run_flag = 0;
uint32_t count = 0;

float targetPWM = 0.0f;
float duty      = 0.f;

const float speed[3]  = {1.0f,
                       0.5f,
                       0.2f};

float speed_value[3][2] = { {HIGH_MOTOR_SPEED, MIDDLE_MOTOR_SPEED},
                            {MIDDLE_MOTOR_SPEED, HIGH_MOTOR_SPEED},
                            {HIGH_MOTOR_SPEED, HIGH_MOTOR_SPEED}
};


// =======================================
// setup
// =======================================

void ADC_setup() {
    // ADC Init  Default: HW triggered by TIM3 counter @ 1msec
    JADC_init(seqCHn[0]);
    JADC_init(seqCHn[1]);

    // ADC channel sequence setting
    JADC_sequence(seqCHn, 2);
}

void DC_setup() {

    // TIM_UI_init(TIM3, M_SEC, 1);
    // TIM_UI_enable(TIM3);

    // right side motor
    PWM_init(PWM_PIN1, M_SEC, 1);
    PWM_period(PWM_PIN1, M_SEC ,1);

    // left side motor
    PWM_init(PWM_PIN2, M_SEC, 1);
    PWM_period(PWM_PIN2, M_SEC ,1);

    GPIO_init(DIRECTION_PIN1, OUTPUT);
    GPIO_otype(DIRECTION_PIN1, OUTPUT_PUSH_PULL);
    GPIO_ospeed(DIRECTION_PIN1, HIGH_SPEED);

    GPIO_write(DIRECTION_PIN1, HIGH);
    GPIO_write(DIRECTION_PIN2, LOW);

    // LED pin
    GPIO_init(LED_PIN, OUTPUT);
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_ospeed(LED_PIN, HIGH_SPEED);
}


void ultrasonic_setup() {
    // PWM configuration ---------------------------------------------------------------------
    PWM_init(TRIG, U_SEC, 1);			// PA_6: Ultrasonic trig pulse
    PWM_period_us(TRIG, 50000);					// PWM of 50ms period. Use period_us()
    PWM_pulsewidth_us(TRIG, 10);		// PWM pulse width of 10us

    // Input Capture configuration -----------------------------------------------------------------------
    ICAP_init(ECHO);    						// PB_6 as input caputre
    ICAP_counter_us(ECHO, 10);   			// ICAP counter step time as 10us
    ICAP_setup(ECHO, 1, IC_RISE);   // TIM4_CH1 as IC1 , rising edge detect
    ICAP_setup(ECHO, 2, IC_FALL);   // TIM4_CH2 as IC2 , falling edge detect
}


void bluetooth_setup() {
    UART1_init();
    UART1_baud(BAUD_9600);
}


// =======================================
// interrupt
// =======================================

void auto_control(uint32_t IR_value[], float ultrasonic_distance){

    //default
    float right_duty = HIGH_MOTOR_SPEED;
    float left_duty  = HIGH_MOTOR_SPEED;

    // right turn
    if (IR_value[0] > IR_THRESHOLD && IR_value[1] < IR_THRESHOLD) {
        right_duty = MIDDLE_MOTOR_SPEED;
    // left turn
    }else if((IR_value[0] < IR_THRESHOLD && IR_value[1] > IR_THRESHOLD)) {
        left_duty = MIDDLE_MOTOR_SPEED;
    }

    // emergency stop
    if (ultrasonic_distance < ULTRASONIC_THRESHOLD) {
        right_duty  = 0.f;
        left_duty   = 0.f;
    }

    if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)

        PWM_duty(PWM_PIN2, left_duty);
        PWM_duty(PWM_PIN1, right_duty);

        clear_UIF(TIM3); 		// Clear UI flag by writing 0
    }

}

void manual_control() {
    return;
}


void IR_control(uint32_t IR_value[]){
    if(is_ADC_OVR())
        clear_ADC_OVR();

    if(is_ADC_JEOC()){		// after finishing sequence
        IR_value[0] = JADC_read(1);
        IR_value[1] = JADC_read(2);
        clear_ADC_JEOC();

    }

}

float ultrasonic_control(){
    // Check for Update interrupt
    if(is_UIF(TIM4)){															  // Update interrupt
        ovf_cnt++; 																  // overflow count
        clear_UIF(TIM4);  														  // clear update interrupt flag
    }
    // Check for TIM4_Ch1 (IC1) Capture Flag (Rising Edge)
    if(is_CCIF(TIM4, 1)){ 												  // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
        time1 = ICAP_capture(TIM4, 1);										  // Capture TimeStart
        clear_CCIF(TIM4, 1);												  // clear capture/compare interrupt flag
    }
    // Check for TIM4_Ch2 (IC2) Capture Flag (Falling Edge)
    else if(is_CCIF(TIM4, 2)){ 											  // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
        time2 = ICAP_capture(TIM4, 2);										  // Capture TimeEnd
        timeInterval = (time2 - time1 + (float)(ovf_cnt * (TIM4->ARR+1))) * 0.01; // (10us * counter pulse -> [msec] unit) Total time of echo pulse
        ovf_cnt = 0;															  // overflow reset
        clear_CCIF(TIM4,2);												  // clear capture/compare interrupt flag
    }
    return get_distance(timeInterval);
}


// get distance from ultrasonic sensor
float get_distance(float timeinterval) {
    return timeinterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
}

