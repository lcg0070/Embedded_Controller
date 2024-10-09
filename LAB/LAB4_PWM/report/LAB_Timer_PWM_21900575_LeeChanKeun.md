# **LAB: Timer & PWM**

> Servo motor and DC motor
>

**Date:** 2024-10-08

**Author/Partner:** Lee Chankeun

**Github:** [go to Github(Link)](https://github.com/lcg0070/Embedded_Controller/tree/main/LAB/LAB3_EXTI_SysTick)

**Demo Video:** [go to youtube(Link)](https://www.youtube.com/watch?v=NPJ26TenSJM)

## **Introduction**

This lab is a program that controls servo motors and DC motors using PWM

### **Requirement**

**Hardware**

- MCU
   - NUCLEO-F411RE
- Actuator/Sensor/Others:
   - 3 LEDs and load resistance
   - RC Servo Motor (SG90)
   - DC motor (5V)
   - DC motor driver(LS9110s)
   - breadboard

**Software**

- PlatformIO installed IDE, EC_HAL library


## **Problem 1: RC servo motor**

An RC servo motor is a tiny and light weight motor with high output power. It is used to control rotation angles, approximately 180 degrees (90 degrees in each direction) and commonly applied in RC car, and Small-scaled robots. The angle of the motor can be controlled by the pulse width (duty ratio) of PWM signal. The PWM period should be set at **20ms or 50Hz**. Refer to the datasheet of the RC servo motor for detailed specifications.  

<img src="https://raw.githubusercontent.com/lcg0070/Embedded_Controller/refs/heads/main/LAB/LAB4_PWM/report/images/RC_servo_motor.avif">   


### **Configuration**

| Type | Port - Pin | Configuration |
| --- | --- | --- |
| **Button** | Digital In (PC13) | Pull-Up |
| **PWM Pin** | AF (PA1) | Push-Pull, Pull-Up, Fast |
| **PWM Timer** | TIM2_CH2 (PA1) | TIM2 (PWM) period: 20msec, Duty ratio: 0.5~2.5msec |
| **Timer Interrupt** | TIM3 | TIM3 Period: 1msec, Timer Interrupt of 500 msec |


### 

### **Circuit Diagram**

> You need to include the circuit diagram
>

[//]: # (<img src="https://raw.githubusercontent.com/lcg0070/Embedded_Controller/refs/heads/main/LAB/LAB4_PWM/report/images/RC_servo_motor.avif" width=50% height=50%>)

### **Discussion**

1. Derive a simple logic to calculate CRR and ARR values to generate x[Hz] and y[%] duty ratio of PWM. How can you read the values of input clock frequency and PSC?

> 1. $$f_{timer} = \frac{f_{\text{clk}}}{(\text{PSC} + 1)}$$
> 2. $$ARR = \frac{f_{\text{timer}}}{\text x} - 1$$
> 3. $$ARR = \frac{f_{\text{clk}}}{(\text{PSC} + 1) \cdot x} - 1$$
> 4. $$CRR = \text{ARR} \times \frac{y}{100}$$
> x is measured in Hz, and y is in percentage. f_clk is the input clock frequency (ex. 84 MHz). Using the known relationship between the clock and the timer, and applying the previously known equations 1 and 2, equations 3 and 4 can be derived. 
>
2. What is the smallest and highest PWM frequency that can be generated for Q1?

   > First, for the Max PWM frequency, you can set it by making the PSC and ARR values 0 to maximize the f_clk value. This will result in the equation f_max = f_clk. Due to the characteristics of the board, the maximum is 100MHz, so 100MHz is the maximum frequency
   > 
   > Secondly, when looking at the datasheet of the Nucleo board, calculating the maximum values for ARR and PSC, both are 16-bit, so the maximum value is 2^16 - 1, which is 65535. Also, when using the LSE value (32.768kHz) instead of PLL and calculating the formula, it results in 0.00000762939453125Hz.

### **Code**

Your code goes here: [ADD Code LINK such as github](https://github.com/ykkimhgu/EC-student/)


```
// /*----------------------------------------------------------------\
// Author           : Lee ChanKeun
// Created          : 10-08-2024
// Modified         : 10-09-2024
// Language/ver     : C in CLION with platformio
//
// Description      : LAB_TimerPWM
// /----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO2.h"
#include "ecSysTick2.h"
#include "ecRCC2.h"
#include "ecTIM2.h"
#include "ecPWM2.h"   // ecPWM2.h
#include "ecEXTI2.h"

// Definition Button Pin & PWM Port, Pin
#define PWM_PIN PA_1
void setup(void);

int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while(1){
    }
}

// Initialiization
void setup(void) {
    RCC_PLL_init();
    SysTick_init();
    
    // set timer
    // set Timer period 50 msec
    TIM_UI_init(TIM3, M_SEC, 50);
    TIM_UI_enable(TIM3);

    // set PWM
    // set PWM period 20 msec
    PWM_init(PWM_PIN, M_SEC, 1);
    PWM_period(PWM_PIN, M_SEC ,20);

    // set EXTI button pin
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_otype(BUTTON_PIN, OUTPUT_PUSH_PULL);

    EXTI_init(BUTTON_PIN, FALL,1);
}

// 2.5ms~0.5ms(180~0)
// 2/18 -> per 10 degree
// duty = (0.5+(1/9)*i)/20
// PWM INTERRUPT

int i=0;                    // parameter for duty calculate
int timer_flag = 0;         // parameter for change of the direction of servo_motor 
uint32_t count = 0;         // parameter for timer clk

void TIM3_IRQHandler(void){
    if(is_UIF(TIM3)){			                            // Check UIF(update interrupt flag)
        count++;
        if(count > 9) {
            PWM_duty(PWM_PIN, (float)(0.5+(1./9.)*i)/20.);  // calculate duty
            count = 0;
            if(timer_flag) i--;
            else i++;;
            if(i < 1 || i>17) timer_flag = !timer_flag;     // change direction
        }
        clear_UIF(TIM3); 		                            // Clear UI flag by writing 0
    }
}

// BUTTON Interrupt
void EXTI15_10_IRQHandler(void) {
    //check pending
    if(is_pending_EXTI(BUTTON_PIN) ) {
        //debouncing
        for(int i=0; i<30000; i++){}

        // reset all parameter
        count = 0;
        i = 0;
        timer_flag = 0;
        
        //clear pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}
```

### **Results**

The following is a photo of the experimental results under different conditions.  
Unfortunately, when calculating the PWM duty ratio as 2.5ms/20ms, it did not stop precisely at 180 degrees, but instead went beyond 180 degrees before stopping.  


| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_0.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_1.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_2.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_2.jpeg?raw=true" width=50% height=50%> |
|:--------------------------------------------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------------------------------------------------------:| 
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_3.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_4.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_5.jpeg?raw=true" width=50% height=50%> |                                                                                                                                                    | 
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_6.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_7.jpeg?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_8.jpeg?raw=true" width=50% height=50%> |                                                                                                                                                    | 
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB3_EXTI_SysTick/report/images/EXTI_9.jpeg?raw=true" width=50% height=50%> |                                                                                                                                                    |                                                                                                                                                    |                                                                                                                                                    | 
>

Add [demo video link](https://github.com/ykkimhgu/course-doc/blob/master/ec-course/lab/link/README.md)

---

## **Problem 2: DC motor**

### **Procedure**

Make a simple program that rotates a DC motor that changes the duty ratio from 25% -->75%--> 25% --> and so on.

The rotating speed level changes every 2 seconds.

By pressing the push button (PC13), toggle from Running and stopping the DC motor

**First, you MUST read** [Tutorial: DC motor driver connection](https://ykkim.gitbook.io/ec/ec-course/tutorial/tutorial-dcmotor-motor-driver-connection)

1. Use the same project.
- Create a new source file named “**LAB_PWM_DCmotor.c”**
- You need to eliminate the other source file that contains `main()` from the project
   - e.g. Eliminate "“**LAB_PWM_RCmotor.c”** from the project

> You MUST write your name on the source file inside the comment section.
>
1. Connect DC motor and DC motor driver.
- PA_0 for the DC motor PWM
- PC_2 for Direction Pin
1. Change DC motor from LOW Speed to HIGH Speed for every 2 seconds
- e.g. 25% -->75%--> 25% --> and so on.
1. When Button is pressed, it should PAUSE or CONTINUE motor run

### **Configuration**

### 

| Function | Port - Pin | Configuration |
| --- | --- | --- |
| **Button** | Digital In (PC13) | Pull-Up |
| **Direction Pin** | Digital Out (PC2) | Push-Pull |
| **PWM Pin** | AF (PA0) | Push-Pull, Pull-Up, Fast |
| **PWM Timer** | TIM2_CH1 (PA0) | TIM2 (PWM) period: **1msec (1kHz)** |
| **Timer Interrupt** | TIM3 | TIM3 Period: 1msec, Timer Interrupt of 500 msec |
|  |  |  |

### **Circuit Diagram**

> You need to include the circuit diagram
>

image

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F38373000%2F192134563-72f68b29-4127-42ac-b064-2eda95a9a52a.png&width=768&dpr=4&quality=100&sign=60f2bbed&sv=1

### **Code**

Your code goes here: [ADD Code LINK such as github](https://github.com/ykkimhgu/EC-student/)

Explain your source code with necessary comments.

Copy

```
// YOUR MAIN CODE ONLY
// YOUR CODE
```

### **Results**

Experiment images and results

> Show experiment images /results
>

Add [demo video link](https://github.com/ykkimhgu/course-doc/blob/master/ec-course/lab/link/README.md)

### **Reference**

Complete list of all references used (github, blog, paper, etc)

Copy

```

```

## **Troubleshooting**

### **1. motor PWM duty ratio for different DIR**

When, DIR=0 duty=0.8--> PWM 0.8 // 실제 모터에 전달되는 pwm

Whe, DIR=1 duty=0.8--> PWM 0.2 // 실제 모터에 전달되는 PWM

- ** a solution ***

Copy

```
float targetPWM;  // pwm for motor input
float duty=abs(DIR-targetPWM); // duty with consideration of DIR=1 or 0

PWM_duty(PWM_PIN, duty);
```

### **2. Motor does not run under duty 0.5**

SOL) Configure motor PWM period as 1kHz

### **3. Check and give different Interrupt Priority**

Check if you have different NVIC priority number for each IRQs

(Option) You can write Troubleshooting section

Copy

```

### 4. Print a string for BT (USART1)
Use `sprintf()`

```c++
#define _CRT_SECURE_NO_WARNINGS    // sprintf 보안 경고로 인한 컴파일 에러 방지
#include <stdio.h>     // sprintf 함수가 선언된 헤더 파일

char BT_string[20]=0;

int main()
{
	sprintf(BT_string, "DIR:%d PWM: %0.2f\n", dir, duty);    // 문자, 정수, 실수를 문자열로 만듦
	USART1_write(BT_string, 20);
	// ...
}
```

https://dojang.io/mod/page/view.php?id=352 **