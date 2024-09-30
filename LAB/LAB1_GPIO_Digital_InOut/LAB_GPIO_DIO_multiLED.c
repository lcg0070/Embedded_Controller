//
// Created by LeeChanKeun on 2024. 9. 10..
//

#include "ecRCC2.h"
#include "ecGPIO2.h"

#define BUTTON_PIN   PC_13
#define LED_PIN1  PA_5
#define LED_PIN2  PA_6
#define LED_PIN3  PA_7
#define LED_PIN4  PB_6

// Initialiization
void setup(void) {
    RCC_HSI_init();
    // initialize the pushbutton pin as an input:
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, PULL_UP);

    // initialize the LED pin as an output:
    GPIO_init(LED_PIN1, OUTPUT);
    GPIO_otype(LED_PIN1, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN1, PULL_DOWN);
    GPIO_ospeed(LED_PIN1, MEDIUM_SPEED);

    GPIO_init(LED_PIN2, OUTPUT);
    GPIO_otype(LED_PIN2, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN2, PULL_DOWN);
    GPIO_ospeed(LED_PIN2, MEDIUM_SPEED);

    GPIO_init(LED_PIN3, OUTPUT);
    GPIO_otype(LED_PIN3, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN3, PULL_DOWN);
    GPIO_ospeed(LED_PIN3, MEDIUM_SPEED);

    GPIO_init(LED_PIN4, OUTPUT);
    GPIO_otype(LED_PIN4, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN4, PULL_DOWN);
    GPIO_ospeed(LED_PIN4, MEDIUM_SPEED);

}

void LED_set1(){
    GPIO_write(LED_PIN1, HIGH);
    GPIO_write(LED_PIN2, LOW);
    GPIO_write(LED_PIN3, LOW);
    GPIO_write(LED_PIN4, LOW);
}

void LED_set2(){
    GPIO_write(LED_PIN1, LOW);
    GPIO_write(LED_PIN2, HIGH);
    GPIO_write(LED_PIN3, LOW);
    GPIO_write(LED_PIN4, LOW);
}

void LED_set3(){
    GPIO_write(LED_PIN1, LOW);
    GPIO_write(LED_PIN2, LOW);
    GPIO_write(LED_PIN3, HIGH);
    GPIO_write(LED_PIN4, LOW);
}

void LED_set4(){
    GPIO_write(LED_PIN1, LOW);
    GPIO_write(LED_PIN2, LOW);
    GPIO_write(LED_PIN3, LOW);
    GPIO_write(LED_PIN4, HIGH);
}

int main(void) {
    setup();
    int buttonState      = 0;
    int LEDState         = 0;

    while(1){
        buttonState = GPIO_read(BUTTON_PIN);
        if(!buttonState) {
            if(LEDState > 3) LEDState = 0;
            while (!GPIO_read(BUTTON_PIN)) {
                switch(LEDState){
                    case(0):
                        LED_set1();
                        break;
                    case(1):
                        LED_set2();
                        break;
                    case(2):
                        LED_set3();
                        break;
                    case(3):
                        LED_set4();
                        break;
                    default:
                        break;
                }
            }
            LEDState++;
        }
    }
}