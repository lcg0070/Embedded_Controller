# LAB : GPIO Digital InOut

**Date** : 2024-09-14

Author : LeeChanKeun

Github : [go to Github(Link)](https://github.com/lcg0070/Embedded_Controller/tree/main/EC2024/LAB/LAB1_GPIO_Digital_InOut)

Demo Video : [go to youtube(Link)](https://www.youtube.com/watch?v=tGBceJLzedQ&list=PLFnQJkfClAZljIpVHo8O4TpGWvpMmlaBk)

# **Introduction**

In this lab, I created HAL drivers for GPIO digital input and output control, and used them to develop a simple program that toggles multiple LEDs with a push button input.

# Requirement

### **Hardware**

- MCU
    - NUCLEO-F411RE
- Actuator/Sensor/Others:
    - LEDs x 3
    - Resistor 330 ohm x 3, breadboard

### **Software**

- PlatformIO, CMSIS, EC_HAL library

# **Problem 0: STM-Arduino**

We are going to create a simple program that turns LED(LD2) on and off by pressing the user button(BT1), using Arduino Syntax

### **GPIO Digital In/Out**

Create a new project under the directory

- The project folder name is “**\LAB_GPIO_DIO_LED”.**

Configures the specified pin to behave either as an input or an output.

Copy

```
pinMode(pin, mode)
```

- pin: the pin number to set the model of.
- mode: INPUT, OUTPUT or INPUT_PULLUP.

> Look up for pinMode() function in arduino reference for detail description.
>

### **Example code**

Open *Arduino IDE* and Create a new program named as ‘**TU_arduino_GPIO_LED_button.ino**’.

Write the following source code: [source code](https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB1_GPIO_Digital_fInOut/LED_toggle/LED_toggle.ino).

Copy

```
const int btnPin = 3;
const int ledPin = 13;

int btnState = HIGH;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(btnPin, INPUT);
}

void loop() {
  btnState = digitalRead(btnPin);

  if (btnState == HIGH)
    digitalWrite(ledPin, LOW);

  else
    digitalWrite(ledPin, HIGH);

}
```

The user button pin is `PC13`, but this pin cannot be used in arduino. So, you should connect `PC13` to `pinName` `D3` by using wire.

![pin_image](https://raw.githubusercontent.com/lcg0070/Embedded_Controller/main/EC2024/LAB/LAB1_GPIO_Digital_InOut/images/image.avif)

Click on **upload** button. Push the reset button(black) and check the performance.

The LED(LD2) should be turned on when the button is pressed.

---

## **Problem 1: Create EC_HAL library**

### **Procedure**

Create the library directory `\repos\EC\lib\`.

Save your header library files in this directory.

> DO NOT make duplicates of library files under each project folders
>

Create your own library for Digital_In and Out : `ecGPIO.h, ecGPIO.c`

- [Download library files from here](https://github.com/lcg0070/Embedded_Controller/tree/main/EC2024/HAL)
- Use the provided `ecRCC2.h` and `ecRCC2.c`
- Modify `ecGPIO2.c`, `ecGPIO2.h`

**ecRCC2.h**

Copy

```
void RCC_HSI_init(void);
void RCC_GPIOA_enable(void);
void RCC_GPIOB_enable(void);
void RCC_GPIOC_enable(void);
```

**ecGPIO2.h**

Copy

```
void GPIO_init(PinName_t pinName, int mode);
void GPIO_write(PinName_t pinName, int Output);
int  GPIO_read(PinName_t pinName);
void GPIO_mode(PinName_t pinName, int mode);
void GPIO_ospeed(PinName_t pinName, int speed);
void GPIO_otype(PinName_t pinName, int type);
void GPIO_pupd(PinName_t pinName, int pupd);
```

- Example code in **ecGPIO2.c**

Copy

```
/* ecGPIO2.c  */

// Input(00), Output(01), AlterFunc(10), Analog(11, reset)
void GPIO_mode(PinName_t pinName, uint32_t mode){
 	GPIO_TypeDef *port;
	unsigned int pin;
	ecPinmap(pinName, &port, &pin);

	port->MODER &= ~(3UL<<(2*pin));
	port->MODER |= mode<<(2*pin);
}
```

## **Problem 2: Toggle LED with Button**

### **Procedure**

1. Create a new project under the directory `\repos\EC\LAB\`
- The project name is “**LAB_GPIO_DIO_LED”.**
- Name the source file as “**LAB_GPIO_DIO_LED.c”**
- Use the [code provided here](https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB1_GPIO_Digital_fInOut/LAB_GPIO_DIO_LED.c).

2. Include your library **ecGPIO2.h, ecGPIO2.c** in `\repos\EC\lib\`.

> You MUST write your name in the top of the source file, inside the comment section.
>

3. Toggle the LED by pushing the button.

- Push button (LED ON), Push Button (LED OFF) and repeat

### **Configuration**

| Button (B1) | LED |
| --- | --- |
| Digital In | Digital Out |
| GPIOC, Pin 13 | GPIOA, Pin 5 |
| PULL-UP | Open-Drain, Pull-up, Medium Speed |

### **Code**

This Code toggles the LEDs using the LED pin PA_5 and the button pin PC_13.


**Sample Code**

```
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
    int buttonState = 0;
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
```
Demo Video : [demo video link](https://www.youtube.com/shorts/S24QNCfzZIM)

### **Discussion**

1. Find out a typical solution for software debouncing and hardware debouncing.
   - Bouncing in circuits refers to the rapid switching between on and off states that occurs when a mechanical switch or button is pressed or released. 
   - ##### Software Debouncing(There are several common methods for software debouncing)
     - Delay method : After detecting a button press, wait for a short period before reading th button state again
     - Multiple sampling: Read the button state multiple times over a short period and only consider it pressed if all samples agree
     - State machine: Implement a state machine that requires the button to be in a stable state for a certain number of consecutive reads
     
   - ##### Hardware solution(Hardware debouncing uses additional components to smooth out the bouncing signals)
     - RC circuit: Add a resistor and capacitor in parallel with the switch. This creates a low-pass filter that smooths out the rapid transitions 
     - Schmitt trigger: Use a Schmitt trigger IC to create a more defined transition between high and low states
     - Flip-flop circuit: Implement a simple flip-flop circuit using discrete components or an IC to latch the button state
       
2. What method of debouncing did this NUCLEO board use for the push-button(B1)?
   - There are pull-up resister (100kohm) and capacitor (100nF). These components debounce the signal.
## 

## **Problem 3: Toggle LED with Button**

### **Procedure**

1. Create a new project under the directory `\repos\EC\LAB\`
- The project name is “**LAB_GPIO_DIO_multiLED”.**
- Name the source file as “**LAB_GPIO_DIO_multiLED.c”**

> You MUST write your name in the top of the source file, inside the comment section.
>
1. Include your library **ecGPIO2.h, ecGPIO2.c** in `\repos\lib\`.
2. Connect 4 LEDs externally with necessary load resistors.
- As Button B1 is Pressed, light one LED at a time, in sequence.
- Example: LED0--> LED1--> …LED3--> …LED0….

### **Configuration**

| Button | LED |
| --- | --- |
| Digital In | Digital Out |
| GPIOC, Pin 13 | PA5, PA6, PA7, PB6 |
| PULL-UP | Push-Pull, Pull-up, Medium Speed |

### **Circuit Image**

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB1_GPIO_Digital_InOut/images/reset_state.jpeg?raw=true" width=50% height=50% alt="circuit_image">

### **Code**
This code is designed to manage state changes in response to button presses. To prevent rapid state changes during continuous button presses, a debouncing mechanism has been implemented using a while loop. This ensures that the state remains stable until the button is released.

```
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
```

### **Results**

Experiment images and results


| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB1_GPIO_Digital_InOut/images/blue_LED.jpeg?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB1_GPIO_Digital_InOut/images/yellow_LED.jpeg?raw=true" width=50% height=50%> |
|:-----------------------------------------------------------------------------------------------------------------------------------------------------------:|:------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB1_GPIO_Digital_InOut/images/green_LED.jpeg?raw=true" width=50% height=50%> |  <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB1_GPIO_Digital_InOut/images/red_LED.jpeg?raw=true" width=50% height=50%>   |

The images provided depict circuit diagrams corresponding to different states of the system. Upon reviewing the demo video, 
it becomes evident that the implemented circuit fully satisfies all the requirements specified in the problem statement.

Demo Video : [demo video link](https://www.youtube.com/watch?v=tGBceJLzedQ)

[//]: # (### **Discussion**)

[//]: # ()
[//]: # (1. Find out a typical solution for software debouncing and hardware debouncing. What method of debouncing did this NUCLEO board use for the push-button&#40;B1&#41;?)

[//]: # ()
[//]: # (> Answer discussion questions)

[//]: # (>)

## **Reference**

```
STMicroelectronics. "UM1724 User manual: STM32 Nucleo-64 boards (MB1136)." STMicroelectronics. https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf (accessed September 14, 2024).
STMicroelectronics. "Description of STM32F4 HAL and low-layer drivers." STMicroelectronics. https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f4-hal-and-ll-drivers-stmicroelectronics.pdf (accessed September 14, 2024).
```

## **Troubleshooting**
```
In systems where state transitions are triggered by button presses, 
the high operating frequency of the NUCLEO board can lead to unintended rapid state changes. 
This phenomenon occurs because the system can detect and process button inputs at a rate far exceeding human interaction speed. 
Consequently, a single button press might be interpreted as multiple inputs, resulting in undesired state transitions.
To mitigate this issue, it is crucial to implement a more conservative approach to state change logic. 
One effective method is the utilization of a while loop construct. 
This technique ensures that the state remains stable until specific conditions are met, typically involving a deliberate release of the button.
```