//
// Created by 이찬근 on 2024. 11. 19..
//

#include "smartFarm_Hani.h"
#include "ecSTM32F4.h"



// =======================================
// Common
// =======================================

void LED_setup() {
    // LED pin
    GPIO_init(LED_PIN, OUTPUT);
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, PULL_UP);
    GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
}


void UART1_setup() {
    UART1_init();
    UART1_baud(BAUD_9600);
}

void UART2_setup() {
    UART2_init();
    UART2_baud(BAUD_9600);
}


void GPIO_toggle(PinName_t pinName) {
    GPIO_TypeDef *Port;
    unsigned int pin;
    ecPinmap(pinName, &Port, &pin);

    if (Port->ODR & (1UL << pin)) {
        Port->BSRR = (uint32_t)1UL << (pin + 16); // Reset the pin
    } else {
        Port->BSRR = (uint32_t)1UL << pin; // Set the pin
    }
}


// =======================================
// Communication Send
// =======================================


void communication_send_setup() {
    RCC_PLL_init();
    SysTick_init();

    LED_setup();
    communication_send_init();
    UART1_setup();
}

#define PIN_INDEX (int)4
PinName_t pin[PIN_INDEX];

void communication_send_init() {
    pin[0]  =   COMMUNICATION_SEND_PINA;
    pin[1]  =   COMMUNICATION_SEND_PINB;
    pin[2]  =   COMMUNICATION_SEND_PINC;
    pin[3]  =   COMMUNICATION_SEND_PIND;

    for(int i = 0; i < PIN_INDEX; i++) {
        GPIO_init(pin[i], INPUT);
        GPIO_pupd(pin[i], PULL_DOWN);
    }
    GPIO_init(LED_PIN, OUTPUT);
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, PULL_DOWN);
    GPIO_ospeed(LED_PIN, MEDIUM_SPEED);
}

void process_button_states(uint8_t *button_state_current, uint8_t *button_state_history) {
    *button_state_current = 0;
    for (int i = 0; i<4; i++) {
        *button_state_current |= (GPIO_read(pin[i]) == HIGH) << i;
    }
    uint8_t changed_buttons = *button_state_current ^ *button_state_history;
    *button_state_history = *button_state_current;

    if (changed_buttons) {
        USART1_write(button_state_current, sizeof(*button_state_current));
        if (*button_state_current) {
            GPIO_write(LED_PIN, HIGH);
        } else {
            GPIO_write(LED_PIN, LOW);
        }
    }
}


// =======================================
// Main Control
// =======================================
void main_setup() {
    RCC_PLL_init();
    SysTick_init();

    // Timer
    TIM_UI_init(TIM3, M_SEC, TIMER_DUTY_MSEC);

    // communication
    communication_recieve_setup();

    // ph sensor
    ph_sensor_setup();
}


//==============
// communication
//==============
void communication_recieve_setup() {
    LED_setup();
    UART2_setup();
    UART1_setup();
}


// BT_Data -> Flags
// 0: AUTO, MANUAL            ON,OFF
// 1: WATER_MOTOR             ON,OFF
// 2: Hydroponic_Nutrients    ON,OFF
// 3: LED, MAIN_PUMP          ON,OFF
static volatile uint8_t BT_Data = 0;
void blutooth_data2flag(uint8_t flags[]) {
    if (is_USART1_RXNE()) {
        BT_Data = USART1_read();
        if(BT_Data == 0) return;
        for (int i = 0; i < 4; i++) {
            if ( BT_Data >> i & 0b1) {
                flags[i] = !flags[i];
            }
        }
    }
}


//============
// ph sensor
//============
PinName_t seqCHn[1] = {PH_SENSOR_PIN};
void ph_sensor_setup() {
    JADC_init(PH_SENSOR_PIN);
    JADC_sequence(seqCHn, 1);
}


float ph_value =0;
float cal_ph() {
    if(is_ADC_OVR()) clear_ADC_OVR();

    if(is_ADC_JEOC()){
        ph_value = JADC_read(1);
        return ph_value2level(ph_value);
    }
    return 0.;
}

float ph_value2level(float ph_value){
    float voltage = ph_value *PH_VOLTAGE_SCALE_FACTOR;
    return 3.5f * voltage + PH_OFFSET;
}




//============
// flag control
//============
void process_farm(uint8_t flags[], uint32_t current_time) {
    static uint8_t previous_mode = 0xFF; // Initialize to an invalid mode
    static uint32_t last_toggle_time = 0;


    if (flags[0] != previous_mode) {
        handle_mode_transition(flags[0], &last_toggle_time);
        previous_mode = flags[0];
    }

    if (flags[0] == HIGH) {
        control_auto_mode(current_time, &last_toggle_time);
    } else {
        control_manual_mode(flags);
    }
}

void handle_mode_transition(uint8_t current_mode, uint32_t *last_toggle_time) {
    if (current_mode == HIGH) { // Switched to AUTO mode
        *last_toggle_time = 0;
        GPIO_write(MAIN_PUMP_PIN, HIGH);
        GPIO_write(FARM_LED_PIN, HIGH);
        GPIO_write(WATER_SUPPLY_PIN, LOW);
        GPIO_write(NUTRIENT_SUPPLY_PIN, LOW);
    }
}

void control_auto_mode(uint32_t current_time, uint32_t *last_toggle_time) {
    GPIO_write(MAIN_PUMP_PIN, HIGH);
    GPIO_write(FARM_LED_PIN, HIGH);

    if (current_time - *last_toggle_time >= TIMER_TICK_COUNT_THRESHOLD) {
        GPIO_toggle(MAIN_PUMP_PIN);
        GPIO_toggle(FARM_LED_PIN);
        *last_toggle_time = current_time;
    }
}

void control_manual_mode(uint8_t flags[]) {
    GPIO_write(MAIN_PUMP_PIN, LOW);
    GPIO_write(FARM_LED_PIN, LOW);

    GPIO_write(WATER_SUPPLY_PIN, flags[1] == HIGH ? HIGH : LOW);
    GPIO_write(NUTRIENT_SUPPLY_PIN, flags[2] == HIGH ? HIGH : LOW);

    if (flags[3] == HIGH) {
        GPIO_write(FARM_LED_PIN, HIGH);
        GPIO_write(MAIN_PUMP_PIN, HIGH);
    } else {
        GPIO_write(FARM_LED_PIN, LOW);
        GPIO_write(MAIN_PUMP_PIN, LOW);
    }
}


