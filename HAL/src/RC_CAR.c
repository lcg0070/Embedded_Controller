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
int dir = FORWARD_MOTOR;

//Ultrasonic parameter//
uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
uint32_t time1 = 0;
uint32_t time2 = 0;



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
void LED_setup() {
    TIM_UI_init(TIM3, M_SEC, 1);
    TIM_UI_enable(TIM3);
    // LED pin
    GPIO_init(LED_PIN, OUTPUT);
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, PULL_UP);
    GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
}


void DC_setup() {

    // right side motor
    PWM_init(PWM_PIN1, M_SEC, 1);
    PWM_period(PWM_PIN1, M_SEC ,1);

    // left side motor
    PWM_init(PWM_PIN2, M_SEC, 1);
    PWM_period(PWM_PIN2, M_SEC ,1);

    GPIO_init(DIRECTION_PIN1, OUTPUT);
    GPIO_otype(DIRECTION_PIN1, OUTPUT_PUSH_PULL);
    GPIO_ospeed(DIRECTION_PIN1, HIGH_SPEED);

    GPIO_init(DIRECTION_PIN2, OUTPUT);
    GPIO_otype(DIRECTION_PIN2, OUTPUT_PUSH_PULL);
    GPIO_ospeed(DIRECTION_PIN2, HIGH_SPEED);

    // Initial value
    GPIO_write(DIRECTION_PIN1, LOW);
    GPIO_write(DIRECTION_PIN2, LOW);

    set_car_speed(STOP_MOTOR_SPEED, STOP_MOTOR_SPEED);
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


void UART1_setup() {
    UART1_init();
    UART1_baud(BAUD_9600);
}

void set_car_speed(float speed_left, float speed_right) {
    PWM_duty(PWM_PIN1, speed_left);
    PWM_duty(PWM_PIN2, speed_right);
}


// =======================================
// DEFAULT
// =======================================

//
// MOTOR CONTROL
//
uint8_t before_mode = MANUAL;
uint8_t initial_state = 0;
void motor_control(uint32_t IR_value[], float ultrasonic_distance, uint8_t bt_data ,uint8_t mode_flag){
    if (before_mode != mode_flag) initial_state = 1;
    switch(mode_flag) {
        case MANUAL:
            motor_manual(bt_data, initial_state);
            initial_state = 0;
        break;
        case AUTO:
            motor_auto(IR_value, ultrasonic_distance, initial_state);
            initial_state = 0;
        break;
        default:
            break;
    }
}

void motor_auto(uint32_t IR_value[], float ultrasonic_distance, uint8_t initial_state) {
    // initialize
    if (initial_state == 1) {
        GPIO_write(DIRECTION_PIN1, 0);
        GPIO_write(DIRECTION_PIN2, 0);
    }
    //default
    set_car_speed(HIGH_MOTOR_SPEED, HIGH_MOTOR_SPEED);
    // // right turn
    if (IR_value[0] > IR_THRESHOLD && IR_value[1] < IR_THRESHOLD) {
        set_car_speed(LOW_MOTOR_SPEED, HIGH_MOTOR_SPEED);
        // left turn
    }else if((IR_value[0] < IR_THRESHOLD && IR_value[1] > IR_THRESHOLD)) {
        set_car_speed(HIGH_MOTOR_SPEED, LOW_MOTOR_SPEED);
    }
    // emergency stop
    if (ultrasonic_distance < ULTRASONIC_THRESHOLD) {
        set_car_speed(STOP_MOTOR_SPEED, STOP_MOTOR_SPEED);
    }
}

void motor_manual(uint8_t bt_data, uint8_t initial_state) {
    // initialize
    if (initial_state == 1) {
        GPIO_write(DIRECTION_PIN1, 0);
        GPIO_write(DIRECTION_PIN2, 0);

        set_car_speed(STOP_MOTOR_SPEED, STOP_MOTOR_SPEED); // Stop car

    }
    // Direction(bt_data, );
    // Direction_display(bt_data);
}

int count = 0;
void led_control(uint8_t mode_flag) {
    if(is_UIF(TIM3)){         // Check UIF(update interrupt flag)
        if (mode_flag == AUTO) {
            count++;
            if (count <= 1000) {

                GPIO_write(LED_PIN, 0);

            }else if(count <= 2000) {

                GPIO_write(LED_PIN, 1);
            }else {
                count = 0;
            }
        }else if( mode_flag == MANUAL) {
            GPIO_write(LED_PIN, 1);
        }
        clear_UIF(TIM3);       // Clear UI flag by writing 0
    }
}

uint8_t mode_toggle(uint8_t mode_flag) {
    if (mode_flag == MANUAL) return AUTO;
    return MANUAL;
}

// =======================================
// AUTO MODE
// =======================================
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
        ovf_cnt = 0; 																  // overflow count
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
        timeInterval = ((float)time2 - (float)time1 + (float)(ovf_cnt * (TIM4->ARR+1))) * 0.01; // (10us * counter pulse -> [msec] unit) Total time of echo pulse
        ovf_cnt = 0;															  // overflow reset
        clear_CCIF(TIM4,2);
    }
    return timeInterval;
}


// get distance from ultrasonic sensor
float get_distance(float timecount) {
    return timecount * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
}




// =======================================
// MANUAL MODE
// =======================================
int forward_ready = 1;
int backward_ready = 0;

//Motor parameter//
float direction = 0.0f;
float targetPWM = 0.0f;
float duty      = 0.f;

void Direction(uint8_t direction, uint8_t arrow_flag) {
    if (arrow_flag) {
        switch (direction) {

            case UP:  // Increase speed
                if ((forward_ready || backward_ready) && duty < HIGH_MOTOR_SPEED) {
                    duty += DUTY_CYCLE_STEP;
                    if (duty >= HIGH_MOTOR_SPEED) {
                        duty = HIGH_MOTOR_SPEED;  // Keep speed at max level
                    }
                    set_car_speed(duty, duty);
                }
            break;

            case DOWN:  // Decrease speed
                if (duty > STOP_MOTOR_SPEED) {
                    duty -= DUTY_CYCLE_STEP;
                    if (duty < STOP_MOTOR_SPEED) duty = STOP_MOTOR_SPEED; // Limit to min
                    set_car_speed(duty, duty);
                }
            break;

            case RIGHT:  // Turn right with reduced speed on one side
                if (duty > STOP_MOTOR_SPEED) {
                    GPIO_write(DIRECTION_PIN1, 0);
                    GPIO_write(DIRECTION_PIN2, 0);
                    PWM_duty(PWM_PIN1, duty * 0.5f); // Reduce right side speed
                    PWM_duty(PWM_PIN2, duty);       // Maintain left side speed
                }
            break;

            case LEFT:  // Turn left with reduced speed on one side
                if (duty > STOP_MOTOR_SPEED) {
                    GPIO_write(DIRECTION_PIN1, 0);
                    GPIO_write(DIRECTION_PIN2, 0);
                    PWM_duty(PWM_PIN1, duty);       // Maintain right side speed
                    PWM_duty(PWM_PIN2, duty * 0.5f); // Reduce left side speed
                }
            break;
        }
    }else{
        switch (direction) {
            case FORWARD:  // Set forward mode and stop the car
                GPIO_write(DIRECTION_PIN1, 0);
                GPIO_write(DIRECTION_PIN2, 0);

                set_car_speed(STOP_MOTOR_SPEED, STOP_MOTOR_SPEED);  //stop
                duty = STOP_MOTOR_SPEED;
                forward_ready = 1;      // Ready to move forward
                backward_ready = 0;
            break;

            case BACKWARD:  // Set backward mode and stop the car
                GPIO_write(DIRECTION_PIN1, 1);
                GPIO_write(DIRECTION_PIN2, 1);
                set_car_speed(HIGH_MOTOR_SPEED, HIGH_MOTOR_SPEED);  //stop
                duty = HIGH_MOTOR_SPEED;
                forward_ready = 0;      // Ready to move forward
                backward_ready = 1;
            break;

            case STOP:  // Stop the car
                set_car_speed(STOP_MOTOR_SPEED, STOP_MOTOR_SPEED); // Set speed to 0
                duty = STOP_MOTOR_SPEED;
                forward_ready = 0;
                backward_ready = 0;
            break;
        }
    }
}

void Direction_display(uint8_t direction, uint8_t arrow_flag){
    if (arrow_flag) {
        switch (direction) {
            case RIGHT:
                USART_write(USART1, (uint8_t*) "RIGHT", 5);
            break;

            case LEFT:
                USART_write(USART1, (uint8_t*) "LEFT ", 5);
            break;

            case UP:
                USART_write(USART1, (uint8_t*) "Speed Up ", 9);
            break;

            case DOWN:
                USART_write(USART1, (uint8_t*) "Speed Down ", 11);
            break;
        }
        switch (direction) {
            case FORWARD:
                USART_write(USART1, (uint8_t*) "Forward", 7);
            break;

            case STOP:
                USART_write(USART1, (uint8_t*) "STOP ", 5);
            break;
        }
   }
   USART_write(USART1, "\r\n", 2);
}