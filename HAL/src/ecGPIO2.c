//
// Created by 이찬근 on 2024. 9. 10..
//

#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO2.h"


//// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)
void GPIO_init(PinName_t pinName, uint32_t mode){
    GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName, &Port, &pin);

    RCC_GPIO_enable(Port);
    GPIO_mode(pinName, mode);
}


void GPIO_write(PinName_t pinName, int Output) {
    GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);
    Port->ODR &= ~(1UL<<(pin));
    Port->ODR |= (Output<<(pin));
}

int GPIO_read(PinName_t pinName){
    GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);

    return (Port->IDR >> (pin)) & 1UL;
}

//// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(PinName_t pinName, uint32_t mode){
    GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);
    Port->MODER &= ~(3UL<<(2*pin));
    Port->MODER |= mode<<(2*pin);
}

//// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(PinName_t pinName, int speed){
    GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);
    Port->OSPEEDR &= ~(3UL<<(2*pin));
    Port->OSPEEDR |= speed<<(2*pin);
}

//// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(PinName_t pinName, int type){
    GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);
    Port->OTYPER &= ~(1UL<<(pin));
    Port->OTYPER |= type<<(pin);
}

//// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(PinName_t pinName, int pupd){
    GPIO_TypeDef * Port;
    unsigned int pin;
    ecPinmap(pinName,&Port,&pin);
    Port->PUPDR &= ~(3UL<<(2 * pin));
    Port->PUPDR |= pupd<<(2 * pin);
}



// SevenSegment_function(with decoder)
#define PIN_INDEX (int)4
PinName_t pin[PIN_INDEX];

void sevensegment_display_init(PinName_t pinNameA, PinName_t pinNameB, PinName_t pinNameC, PinName_t pinNameD) {
    pin[0]  =   pinNameA;
    pin[1]  =   pinNameB;
    pin[2]  =   pinNameC;
    pin[3]  =   pinNameD;

    for(int i = 0; i < PIN_INDEX; i++) {
        GPIO_init(pin[i], OUTPUT);
        GPIO_otype(pin[i], OUTPUT_PUSH_PULL);
        GPIO_pupd(pin[i], NO_PULLUP_PULLDOWN);
        GPIO_ospeed(pin[i], MEDIUM_SPEED);
    }
    GPIO_pupd(BUTTON_PIN, PULL_UP);
}

void sevensegment_display(uint8_t  num) {
    if (num > 9) num = 0;
    for(int i=0; i < PIN_INDEX; i++) {
        GPIO_write(pin[i], (num >> i) & 1UL);
    }
}

// SevenSegment_function(without decoder)
#define PIN_INDEX_NODECODER (int)8
PinName_t pin_nodecoder[PIN_INDEX_NODECODER] = {
    PB_10,
    PA_8,
    PA_9,
    PC_7,
    PB_6,
    PA_7,
    PA_6,
    PA_5
};

const uint8_t decode_states[10] = {
    0b00000011, // DECODE_STATE_0
    0b10011111, // DECODE_STATE_1
    0b00100101, // DECODE_STATE_2
    0b00001101, // DECODE_STATE_3
    0b10011001, // DECODE_STATE_4
    0b01001001, // DECODE_STATE_5
    0b11000001, // DECODE_STATE_6
    0b00011111, // DECODE_STATE_7
    0b00000001, // DECODE_STATE_8
    0b00011001  // DECODE_STATE_9
};

void sevensegment_decoder_init(void) {
    for(int i = 0; i < PIN_INDEX_NODECODER; i++) {
        GPIO_init(pin_nodecoder[i], OUTPUT);
        GPIO_otype(pin_nodecoder[i], OUTPUT_PUSH_PULL);
        GPIO_pupd(pin_nodecoder[i], NO_PULLUP_PULLDOWN);
        GPIO_ospeed(pin_nodecoder[i], MEDIUM_SPEED);
    }
    GPIO_pupd(PC_13, PULL_UP);
}

void sevensegment_decoder(uint8_t  num) {
    if (num > 9) num = 0;
    uint8_t state = decode_states[num];
    for(int i=0; i < PIN_INDEX_NODECODER; i++) {
        GPIO_write(pin_nodecoder[i], (state >> i) & 1UL);
    }
}