# LAB:EXTI&SysTick


**Date:** 2024-10-01

**Author:** Lee Chankeun

**Github:** [go to Github(Link)](https://github.com/lcg0070/Embedded_Controller/tree/main/LAB/LAB3_EXTI_SysTick)

**Demo Video:** [go to youtube(Link)](https://www.youtube.com/watch?v=NPJ26TenSJM)

## **Introduction**

This lab is about two simple programs using interrupt:

(1) displaying the number counting from 0 to 9 with Button Press

(2) counting at a rate of 1 second

### **Requirement**

#### **Hardware**

- MCU
    - NUCLEO-F411RE
- Actuator/Sensor/Others:
    - 4 LEDs and load resistance
    - 7-segment display(5101ASR)
    - Array resistor (330 ohm)
    - breadboard

#### **Software**

- PlatformIO installed IDE, CMSIS, EC_HAL library

---

## **Problem 1: Counting numbers on 7-Segment using EXTI Button**

### **Configuration**

| Digital In for Button (B1) | Digital Out for 7-Segment decoder |
| --- | --- |
| Digital In | Digital Out |
| PC13 | PA7, PB6, PC7, PA9 |
| PULL-UP | Push-Pull, No PullUp-PullDown, Medium Speed |

### **Circuit Diagram**


> <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/exti_diagram.png?raw=true" width=50% height=50%>
>

### **Discussion**

1. We can use two different methods to detect an external signal: polling and interrupt. What are the advantages and disadvantages of each approach?

> Polling and interrupt have a big difference in the order of progress of the code.  
> The pros and cons are divided according to this method of progress  
> Polling  
> - Advantages
>   - Good readability
>   - No overheading
>   - Can be more predictable and deterministic
> - Disadvantages
>   - Wastes CPU cycles checking for events that may not occur frequently
>   - Can cause delays or missed data if polling frequency is too low
>   - Blocks other tasks while polling loop is running 
>   - Less power efficient, as CPU is constantly active
>   
> Interrupt  
> - Advantages
>   - More efficient use of CPU resources 
>   - Don't use the CPU for unnecessary tasks 
>   - More responsive to external events
> - Disadvantages
>   - More complex to implement correctly(priority)  
>   - The code malfunctions if the priority is not made correctly in the context of an interrupt

2. What would happen if the EXTI interrupt handler does not clear the interrupt pending flag? Check with your code

   > The interrupt will be triggered again immediately after the current handler finishes executing. This can lead to the interrupt handler being called repeatedly in rapid succession.  
   > This can occur "Interrupt storm", preventing other code from executing properly  
   >  ```
   >  if(is_pending_EXTI(BUTTON_PIN) ) {
   >     //debouncing
   >     for(int i=0; i<30000; i++){}
   >     
   >     //displaying function    
   >     sevensegment_display(cnt % 10);
   >     cnt++;
   >     if (cnt > 9) cnt = 0;
   >     
   >     //clear pending
   >     clear_pending_EXTI(BUTTON_PIN);
   > }
   >   ```
   > When you see the above code, have to meet the pending condition to execute the interrupt statement, but if the condition does not end, it will change to a code that only executes the interrupt according to the priority.
   In that case, the meaning will change to the code that is continuously executed, not according to the event. It is more advantageous to write the code in a polling manner
    
   
### **Code**
Initialize CLOCK, GPIO, and EXTI register values through setup(void).  
Using the ```EXTI15_10_IRQHandler (void)``` function, the led of the 7th segment was turned on.  
The trigger condition was created using ```is_pending_EXTI (BUTTON_PIN)```.  
The condition was initialized using ```clear_pending_EXTI (BUTTON_PIN)```.  
Debouncing was performed using ```for(int i=0; i<30000; i++){}```.
7-segment led was adjusted using ```sevensegment_display(cnt % 10); ```

```
/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lee ChanKeun
Created          : 10-01-2024
Modified         : 10-01-2024
Language/ver     : C in CLION with platformio

Description      : EXTI(7-segment lab)
/----------------------------------------------------------------*/


#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecRCC2.h"
#include "ecEXTI2.h"

void setup(void);

static unsigned int cnt = 0;

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ---------------------------------------------------------
    while(1){}
}

// Interrupt(Button input) ----------------------------------------------------
void EXTI15_10_IRQHandler(void) {
    //check pending
    if(is_pending_EXTI(BUTTON_PIN) ) {
        //debouncing
        for(int i=0; i<30000; i++){}
        
        //displaying function    
        sevensegment_display(cnt % 10);
        cnt++;
        if (cnt > 9) cnt = 0;
        
        //clear pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}

// Initialiization
void setup(void)
{
    RCC_HSI_init();
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
    EXTI_init(BUTTON_PIN, FALL,13);
}
```


### **Results**
Below is a picture of each situation

| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_0.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_1.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_2.jpeg?raw=true" width=50% height=50%> |
|:--------------------------------------------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------------------------------------------------------:|
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_3.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_4.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_5.jpeg?raw=true" width=50% height=50%> |
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_6.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_7.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_8.jpeg?raw=true" width=50% height=50%> |
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_9.jpeg?raw=true" width=50% height=50%> |                                                                                                                                                    |                                                                                                                                                    |

### [demo video link](https://www.youtube.com/watch?v=NPJ26TenSJM)

## **Problem 2: Counting numbers on 7-Segment using SysTick**

Display the number 0 to 9 on the 7-segment LED at the rate of 1 sec. After displaying up to 9, then it should display ‘0’ and continue counting.

When the button is pressed, the number should be reset ‘0’ and start counting again.


### **Configuration**

| Digital In for Button (B1) | Digital Out for 7-Segment decoder |
| --- | --- |
| Digital In | Digital Out |
| PC13 | PA7, PB6, PC7, PA9 |
| PULL-UP | Push-Pull, No Pull-up-Pull-down, Medium Speed |

### **Circuit Diagram**

> <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/exti_diagram.png?raw=true" width=50% height=50%>
>

### **Code**
Initialize CLOCK, GPIO, EXTI and SysTick register values through setup(void).  
Using the ```EXTI15_10_IRQHandler (void)``` function, the led of the 7th segment set to 0.  
The trigger condition was created using ```is_pending_EXTI (BUTTON_PIN)```.  
The condition was initialized using ```clear_pending_EXTI (BUTTON_PIN)```.
7-segment led was adjusted using ```sevensegment_display(cnt % 10); ```
Time trigger condition was created using ```delay_ms(1000); ```
Time condition was initialized using ```SysTick_reset();```.

```
/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lee ChanKeun
Created          : 10-01-2024
Modified         : 10-01-2024
Language/ver     : C in CLION with platformio

Description      : SysTick(with 7-segment lab)
/----------------------------------------------------------------*/


#include "stm32f4xx.h"
#include "ecGPIO2.h"
#include "ecSysTick2.h"
#include "ecEXTI2.h"

void setup(void);

unsigned int cnt = 0;
int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ---------------------------------------------------------
    while(1) {
        sevensegment_display(cnt % 10);
        delay_ms(1000);
        cnt++;
        if (cnt > 9) cnt = 0;
        SysTick_reset();
    }
}

// Button Trigger -------------------------------------------------------------
void EXTI15_10_IRQHandler(void) {
    if(is_pending_EXTI(BUTTON_PIN) ) {
        cnt = 0;
        // segment display
        sevensegment_display(cnt % 10);
        clear_pending_EXTI(BUTTON_PIN);
    }
}

// Initialiization
void setup(void)
{
    RCC_PLL_init();
    SysTick_init();
    GPIO_init(BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    sevensegment_display_init(PA_7, PB_6, PC_7, PA_9);  // Decoder input A,B,C,D
    EXTI_init(BUTTON_PIN, FALL,13);
}
```

### **Results**
Below is a picture of each situation

| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_0.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_1.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_2.jpeg?raw=true" width=50% height=50%> |
|:-----------------------------------------------------------------------------------------------------------------------------------------------------:|:-----------------------------------------------------------------------------------------------------------------------------------------------------:|:-----------------------------------------------------------------------------------------------------------------------------------------------------:|
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_3.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_4.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_5.jpeg?raw=true" width=50% height=50%> |
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_6.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_7.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_8.jpeg?raw=true" width=50% height=50%> |
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/Systick_9.jpeg?raw=true" width=50% height=50%> |                                                                                                                                                       |                                                                                                                                                       |

### [demo video link](https://www.youtube.com/watch?v=NPJ26TenSJM)

## **Reference**

```
STMicroelectronics. "UM1724 User manual: STM32 Nucleo-64 boards (MB1136)." STMicroelectronics. https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf (accessed October 1, 2024).
STMicroelectronics. "Description of STM32F4 HAL and low-layer drivers." STMicroelectronics. https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f4-hal-and-ll-drivers-stmicroelectronics.pdf (accessed October 1, 2024).
stackoverflow.com. "Clearing pending EXTI interrupt in stm32f10." https://stackoverflow.com/questions/61533331/clearing-pending-exti-interrupt-in-stm32f103.(accessed October 1, 2024).
```

## **Troubleshooting**
> When pressing Button, it was found that the number exceeded 2-3 at a time, so debouncing was conducted with the while statement.  
In the case of problem 2, even if the trigger is made several times by button, it is the purpose of resetting to zero, so debouncing was conducted only on Problem 1
