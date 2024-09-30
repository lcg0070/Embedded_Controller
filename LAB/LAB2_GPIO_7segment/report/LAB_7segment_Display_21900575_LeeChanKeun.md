# LAB: GPIO Digital InOut 7-segment

**Date:** 2024-09-21

Author : LeeChanKeun 

Github : [go to Github(Link)](https://github.com/lcg0070/Embedded_Controller/tree/main/EC2024/LAB/LAB2_GPIO_7segment)

Demo Video : [go to youtube(Link)](https://www.youtube.com/watch?v=D2mtUNAxKnc)

PDF version : [go to PDF(Link)](https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/LAB_7segment_Display_21900575_LeeChanKeun.pdf)

## **Introduction**

In this lab, I made a program that increases the number of 7-segment diode each time I press a button.

The composition of the file is as follows
- LAB Report (*.pdf)
- Zip source files(LAB***.c, ecRCC2.h, ecGPIO2.h etc...).

### **Requirement**

**Hardware**

- MCU
    - NUCLEO-F411RE
- Actuator/Sensor/Others:
    - 7-segment display(5101ASR)
    - Array resistor (330 ohm)
    - decoder chip(74LS47)
    - breadboard

**Software**

- PlatformIO installed IDE, CMSIS, EC_HAL library

## **Exercise**

Fill in the table

| **Port/Pin**   | **Description**              | **Register setting**                     |
|----------------|------------------------------|------------------------------------------|
| Port A Pin 5   | Clear Pin5 mode              | GPIOA->MODER &= ~(3<<(5*2))              |
| Port A Pin 5   | Set Pin5 mode = Output       | GPIOA->MODER \|= 1<<(5*2)                |
| Port A Pin 6   | Clear Pin6 mode              | GPIOA->MODER &= ~(3<<(6*2))              |
| Port A Pin 6   | Set Pin6 mode = Output       | GPIOA->MODER \|= 1<<(5*2)                |
| Port A Pin Y   | Clear PinY mode              | GPIOA->MODER &= ~(3<<(Y*2))              |
| Port A Pin Y   | Set PinY mode = Output       | GPIOA->MODER \|= 1<<(Y*2)                |
| Port A Pin 5~9 | Clear Pin5~9 mode            | GPIOA->MODER &=~(0b1111111111 << (5*2))  |
|                | Set Pin5~9 mode = Output     | GPIOA->MODER \|= (0b0101010101 << (5*2)) |
| Port X Pin Y   | Clear Pin Y mode             | GPIOX->MODER &= ~(3<<(Y*2))              |
|                | Set Pin Y mode = Output      | GPIOX->MODER \|= 1<<(Y*2)                |
| Port A Pin5    | Set Pin5 otype=push-pull     | GPIOA->OTYPER &= ~(1<<(5))               |
| Port A PinY    | Set PinY otype=push-pull     | GPIOA->OTYPER &= ~(1<<(Y))               |
| Port A Pin5    | Clear Pin5 ospeed            | GPIOA->OSPEEDR &= ~(1<<(5))              |
|                | Set Pin5 ospeed=Fast         | GPIOX->MODER \|= 1<<(Y*2)                |
| Port A PinY    | Clear PinY ospeed            | GPIOA->OSPEEDR &= ~(1<<(Y))              |
|                | Set PinY ospeed=Fast         | GPIOX->OSPEEDR \|= 1<<(Y)                |
| Port A Pin 5   | Set Pin5 PUPD=no pullup/down | GPIOA->PUPDR &= ~(3<<(5*2))              |
| Port A Pin Y   | Set PinY PUPD=no pullup/down | GPIOA->PUPDR &= ~(3<<(Y*2))              |

---

## **Problem 0: Connection of 7-Segment Display and Decoder**

### **Procedure**

Review 7-segment Decoder and Display from Digital Logic lecture.

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/images/7_decoder.png?raw=true" width=50% height=50%>
<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/images/7_seg.png?raw=true" width=50% height=50%>
 

### **1. 7-segment display connection**

First, connect the common anode 7-segment display with the given array resistors.

Then, apply VCC and GND to the 7-segment display.

Check that all LEDs of 7-segment work properly

- Give 'H' signal to each 7-segment pin of 'a'~'g' . Observe if that LED is turned ON or OFF
- Example: Connect VCC to all 'a'~'g' pins

### **2. BCD 7-segment decoder connection**

The popular BCD 7-segment decoder chip is **74LS47.** With the BCD chip, you need only 4 DOUT pins of MCU to display from 0 to 9.

Connect the decoder chip (**74LS47**) on the bread board.

<img src="https://raw.githubusercontent.com/lcg0070/Embedded_Controller/refs/heads/main/EC2024/LAB/LAB2_GPIO_7segment/images/image.avif" width="50%" height="50%">

Then, Check that the decoder chip works properly

- Supply a combination of VCC/GND to the pins of 'A'~'D' of the decoder.
- Check if the 7-segment LED display shows the correct number

### **Connection Diagram**

Circuit diagram

> <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/images/diagram.png?raw=true" width="50%" height="50%">


### **Discussion**

1. Draw the truth table for the BCD 7-segment decoder with the 4-bit input.


| Input |   |   |   |     |      |         | Output |     |     |     ||||
|-----|---|---|---|-----|------|---------|--------|-----|-----|-----|----|----|----|
| D   | C | B | A | ~LT | ~RBI | ~BI/RBO | ~a     | ~b  | ~c  | ~d  | ~e | ~f | ~g
| L   | L | L | L | H   | H    | H       | L      | L   | L   | L   | L  | L  | H  |
| L   | L | L | H | H   | H    | H       | H      | L   | L   | H   | H  | H  | H  |
| L   | L | H | L | H   | H    | H       | L      | L   | H   | L   | L  | H  | L  |
| L   | L | H | H | H   | H    | H       | L      | L   | L   | L   | H  | H  | L  |
| L   | H | L | L | H   | H    | H       | H      | L   | L   | H   | H  | L  | L  |
| L   | H | L | H | H   | H    | H       | L      | H   | L   | L   | H  | L  | L  |
| L   | H | H | L | H   | H    | H       | H      | H   | L   | L   | L  | L  | L  |
| L   | H | H | H | H   | H    | H       | L      | L   | L   | H   | H  | H  | H  |
| H   | L | L | L | H   | H    | H       | L      | L   | L   | L   | L  | L  | L  |
| H   | L | L | H | H   | H    | H       | L      | L   | L   | H   | H  | L  | L  |

2. What are the common cathode and common anode of 7-segment display?

> Common Cathode:  
> All the cathodes (negative terminals) of the LEDs are connected together to a common ground.
The individual anodes are controlled with positive voltage to light up segments.
To light a segment, you apply a positive voltage to its anode.
Suited for ground-referenced logic systems.   
> Common Anode:   
> All the anodes (positive terminals) of the LEDs are connected together to a common positive voltage.
The individual cathodes are controlled with negative voltage (ground) to light up segments.
To light a segment, you connect its cathode to ground.
Suited for positive logic systems.

3. Does the LED of a 7-segment display (common anode) pin turn ON when 'HIGH' is given to the LED pin from the MCU?

> No. As described above, if a high is granted, the 7-segment display will be turned off. 

---

## **Problem 1: Display a Number with Button Press**

### **Procedure**

1. Create a new project under the directory `\repos\EC\LAB\LAB_GPIO_7segment`
- The project name is “**LAB_GPIO_7segment”.**
- Create a new source file named as “**LAB_GPIO_7segment.c”**
- Refer to the [sample code](https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/LAB_GPIO_7segment.c)


2. Include your updated library in `\repos\EC\lib\` to your project.
- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**
3. Declare and Define the following functions in your library


```
void sevensegment_display_init(PinNames_t pinNameA, PinNames_t pinNameB, PinNames_t pinNameC, PinNames_t pinNameD);
void sevensegment_display(uint8_t  num);
```

- num: 0 to 9 only (unsigned)
1. Configure and connect the MCU to the circuit
2. First, check that every number, 0 to 9, can be displayed properly
3. Then, create a code that increase the displayed number from 0 to 9 with each button press.
    - After the number '9', it should start from '0' again.

### **Configuration**

Configure the MCU

| Digital In for Button (B1) | Digital Out for 7-Segment |
| --- | --- |
| Digital In | Digital Out |
| PC13 | PA7, PB6, PC7, PA9 |
| PULL-UP | Push-Pull, No Pull-up-Pull-down, Medium Speed |

### **Code**

[**Github Link**](https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/LAB_GPIO_7segment.c)


```
#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"

#define LED_PIN PA_5
#define BUTTON_PIN PC_13

void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();
    unsigned int cnt = 0;

    // Inifinite Loop ----------------------------------------------------------
    while(1){
        sevensegment_display(cnt % 10);

        if(GPIO_read(BUTTON_PIN) == 0) {
            cnt++;
            while (!GPIO_read(BUTTON_PIN)) {}
        }
        if (cnt > 9) cnt = 0;
    }
}


// Initialiization
void setup(void)
{
    RCC_HSI_init();
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
}
```
 
> It was made to increase the cnt variable by 1 each time the button was pressed.  
    The ABCD value connected to the decoder was adjusted according to the cnt value.      
    The address value entering the A,B,C,D pins was declared as a pointer array, and the output value was adjusted according to the index.  
> The code below is used to turn on, off the 7-segment display. This code is written faster and more concisely, unlike the switch case statement.
> - for(int i=0; i < PIN_INDEX; i++) {  
       GPIO_write(pin[i], (num >> i) & 1UL);  
    }


### **Results**

**Experiment images and results**  

The image below shows a command corresponding to 7 applied to the decoder and is a photograph displayed on the 7th segment display
<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/images/7_on.jpeg?raw=true" width=50% height=50%>


Add [demo video link](https://www.youtube.com/watch?v=D2mtUNAxKnc)

### **Discussion**

1. It was confirmed that a slight red color remained like an afterimage whenever the state of the LED was transferred. It was confirmed that the LED was turned on because the residual current remained, and I think we should find a way to solve it as other states change.  
2. Time debouncing was conducted by putting a while statement in the middle of the code, and it was confirmed that there were phenomena that exceeded the blink and the next number depending on the MCU status, and when the switch was attached, it was changed to a condition statement according to the state and modified to a more stable code suitable for the time of the circuit.

---

## **Problem 2: Program BCD-7-segment decoder**

Instead of using the decoder chip, we are going to make the 7-segment decoder with the MCU programming.

> Do not use the 7-segment decoder for this problem

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/images/7_seg_with_decoder.png?raw=true" width=50% height=50%>

### **Procedure**

1. Use the same project and source file.
2. Include your updated library in `\repos\EC\lib\` to your project.
- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**
1. Declare and Define the following functions in your library

Copy

```
void sevensegment_decoder_init(void);
void sevensegment_decoder(uint8_t  num);
```

- num: 0 to 9 only (unsigned)
1. Configure and connect the MCU to the circuit
2. First, check that every number, 0 to 9, can be displayed properly
3. Then, create a code that increases the displayed number from 0 to 9 with each button press.
    - After the number '9', it should start from '0' again.

### **Configuration**

Configure the MCU

| Digital In for Button (B1) | Digital Out for 7-Segment |
| --- | --- |
| Digital In | Digital Out |
| PC13 | PA5, PA6, PA7, PB6, PC7, PA9, PA8, PB10
('a'~'h', respectively) |
| PULL-UP | Push-Pull, No Pull-up-Pull-down, Medium Speed |

### **Code**

Code Link : [**Github Link**](https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/LAB_GPIO_7segment_without_decoder.c)

 
> In the header file, an array of 8 bits in length of 10 corresponding to each state was created and the code was written to assign to each corresponding bit. Except for the exceptions, the operation was minimized by not using a conditional statement.

Code

```
#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"

#define LED_PIN PA_5
#define BUTTON_PIN PC_13

void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();
    unsigned int cnt = 0;

    // Inifinite Loop ----------------------------------------------------------
    while(1){
        sevensegment_decoder(cnt % 10);

        if(GPIO_read(BUTTON_PIN) == 0) {
            cnt++;
            while (!GPIO_read(BUTTON_PIN)) {}
        }
        if (cnt > 9) cnt = 0;
    }
}


// Initialiization
void setup(void)
{
    RCC_HSI_init();
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    sevensegment_decoder_init();
}
```

### **Connection Diagram**

Circuit diagram

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/images/diagram2.png?raw=true" width=50% height=50%>

### **Results**

Experiment images and results  
The picture below is a circuit without a decoder, and the number increased when the switch is pressed.

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/EC2024/LAB/LAB2_GPIO_7segment/images/7_seg_without_decoder.jpeg?raw=true" width=50% height=50%>

Add [demo video link](https://www.youtube.com/watch?v=D2mtUNAxKnc)

### **Discussion**

It could be seen that the circuit worked very well, and the code was simplified and written using array.

---

## **Reference**


```
STMicroelectronics. "UM1724 User manual: STM32 Nucleo-64 boards (MB1136)." STMicroelectronics. https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf (accessed September 22, 2024).
STMicroelectronics. "Description of STM32F4 HAL and low-layer drivers." STMicroelectronics. https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f4-hal-and-ll-drivers-stmicroelectronics.pdf (accessed September 22, 2024).
Electronicsforu.com. "7 Segment Display Pinout, Codes, Working, and Interfacing." CD-Team. https://www.electronicsforu.com/resources/7-segment-display-pinout-understanding.
```

---

## **Troubleshooting**
In general, it was confirmed that when a time is delayed, a problem occurs when the input comes in faster than the time to delay. Therefore, rather than putting time on the event of pressing the switch, I wrote the logic according to the state to show faster reactivity.  
```
while (!GPIO_read(BUTTON_PIN)) {}
```