//
// Created by 이찬근 on 2024. 9. 10..
//

#include "ecRCC2.h"
#include "ecGPIO2.h"

#define LED_PIN     PA_5
#define BUTTON_PIN  PC_13

// Initialiization
void setup(void) {
    RCC_HSI_init();
    // initialize the pushbutton pin as an input:
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_pupd(BUTTON_PIN, PULL_UP);

    // initialize the LED pin as an output:
    GPIO_init(LED_PIN, OUTPUT);
    GPIO_otype(LED_PIN, OUTPUT_PUSH_PULL);
    GPIO_pupd(LED_PIN, PULL_UP);
    GPIO_ospeed(LED_PIN, MEDIUM_SPEED);

}

int main(void) {
    setup();
    int buttonState=0;
    int led_state = 1;
    while(1){
        buttonState = GPIO_read(BUTTON_PIN);

        if (!buttonState) {
            led_state = !led_state;
            while (!GPIO_read(BUTTON_PIN)) {
                GPIO_write(LED_PIN, led_state);
            }
        }
    }
}