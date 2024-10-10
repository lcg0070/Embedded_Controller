# **LAB: Timer & PWM**

> Servo motor and DC motor
>

**Date:** 2024-10-08

**Author/Partner:** Lee Chankeun

**Github:** [go to Github(Link)](https://github.com/lcg0070/Embedded_Controller/tree/main/LAB/LAB4_PWM)

**Demo Video:** [go to youtube(Link)](https://youtu.be/7i9oOtYHT6E)

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

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/servo_diagram.png?raw=true">

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

Initialize TIMER, GPIO, EXTI and PWM register values through ```setup(void)```.  
Using the ```TIM3_IRQHandler (void)``` function, CLK counts, and ```PWM_duty()``` changes.  
The trigger condition was created using ```count``` by TIMER.  
The condition was initialized using ```clear_pending_UIF (TIM3)```.

EXTI Interrupt was also used by```EXTI15_10_IRQHandler (void)``` function.
Reset parameter ```count, i, timer_flag``` when BUTTON_PIN is pushed and ``` is_pending_EXTI (BUTTON_PIN)```is activated

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


|  <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/0_servo.png?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/10_servo.png?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/20_servo.png?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/30_servo.png?raw=true" width=50% height=50%>  |
|:-------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------------------:| 
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/40_servo.png?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/50_servo.png?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/60_servo.png?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/70_servo.png?raw=true" width=50% height=50%>  | 
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/80_servo.png?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/90_servo.png?raw=true" width=50% height=50%>  | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/100_servo.png?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/110_servo.png?raw=true" width=50% height=50%> | 
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/120_servo.png?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/130_servo.png?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/140_servo.png?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/150_servo.png?raw=true" width=50% height=50%> |
| <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/160_servo.png?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/170_servo.png?raw=true" width=50% height=50%> | <img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/180_servo.png?raw=true" width=50% height=50%> |


---

## **Problem 2: DC motor**

This is a problem about controlling a DC motor where pressing a button reverses the direction, and the output alternates between 25% and 75% every 2 seconds.

### **Configuration**

### 

| Function | Port - Pin | Configuration |
| --- | --- | --- |
| **Button** | Digital In (PC13) | Pull-Up |
| **Direction Pin** | Digital Out (PC2) | Push-Pull |
| **PWM Pin** | AF (PA0) | Push-Pull, Pull-Up, Fast |
| **PWM Timer** | TIM2_CH1 (PA0) | TIM2 (PWM) period: **1msec (1kHz)** |
| **Timer Interrupt** | TIM3 | TIM3 Period: 1msec, Timer Interrupt of 500 msec |

### **Circuit Diagram**

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/DC_diagram.png?raw=true">

### **Code**

Initialize TIMER, GPIO, EXTI and PWM register values through ```setup(void)```.  
Using the ```TIM3_IRQHandler (void)``` function, CLK counts, and ```PWM_duty()``` changes.  
The trigger condition was created using ```count``` by TIMER.  
The condition was initialized using ```clear_pending_UIF (TIM3)```.

EXTI Interrupt was also used by```EXTI15_10_IRQHandler (void)``` function.
Change ```run_flag```parameter when ```BUTTON_PIN``` is pushed. This can stop and start the motor
```duty = fabs(DIR - targetPWM)``` function is used to maintain the output of the motor, regardless direction.



```
/*----------------------------------------------------------------\
Author           : Lee ChanKeun
Created          : 10-08-2024
Modified         : 10-09-2024
Language/ver     : C in CLION with platformio

Description      : LAB_PWM_DCmotor
/----------------------------------------------------------------*/

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
#define PWM_PIN         PA_0
#define DIRECTION_PIN   PC_2

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

    TIM_UI_init(TIM3, M_SEC, 500);
    TIM_UI_enable(TIM3);

    // PWM
    PWM_init(PWM_PIN, M_SEC, 1);
    PWM_period(PWM_PIN, M_SEC ,1);

    // Button pin
    GPIO_init(BUTTON_PIN, INPUT);
    GPIO_otype(BUTTON_PIN, OUTPUT_PUSH_PULL);

    EXTI_init(BUTTON_PIN, FALL,1);

    GPIO_init(DIRECTION_PIN, OUTPUT);
    GPIO_otype(DIRECTION_PIN, OUTPUT_PUSH_PULL);
    GPIO_ospeed(DIRECTION_PIN, HIGH_SPEED);

    PWM_duty(PWM_PIN, 0.25);
}



// ================================================
// change State of motor
// ================================================
// PWM INTERRUPT
int run_flag = 1;
uint32_t count = 0;

float targetPWM = 0.25f;  // pwm for motor input
float DIR = 0.f;
float duty; // duty with consideration of DIR=1 or 0

void TIM3_IRQHandler(void){
    if(!run_flag) return;
    if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
        if(count > 3) {
            targetPWM = fabs(1.f - targetPWM);
            duty = fabs(DIR - targetPWM);
            PWM_duty(PWM_PIN, duty);
            count = 0;
        }
        count++;
        clear_UIF(TIM3); 		// Clear UI flag by writing 0
    }
}

// BUTTON Interrupt
void EXTI15_10_IRQHandler(void) {
    //check pending
    if(is_pending_EXTI(BUTTON_PIN) ) {
        //debouncing
        for(int i=0; i<30000; i++){}

        run_flag = !run_flag;
        if(!run_flag) {
            duty = 0;
            PWM_duty(PWM_PIN, duty);
        }else {
            duty = fabs(DIR - targetPWM);
            PWM_duty(PWM_PIN, duty);
            count = 0;
        }

        //clear pending
        clear_pending_EXTI(BUTTON_PIN);
    }
}
```

### **Results**

The following is a photo of the experimental result.

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/dc_result.png?raw=true">


### **Reference**

```
STMicroelectronics. "UM1724 User manual: STM32 Nucleo-64 boards (MB1136)." STMicroelectronics. https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf (accessed October 10, 2024).
STMicroelectronics. "Description of STM32F4 HAL and low-layer drivers." STMicroelectronics. https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f4-hal-and-ll-drivers-stmicroelectronics.pdf (accessed October 10, 2024).
stackoverflow.com. "Clearing pending EXTI interrupt in stm32f10." https://stackoverflow.com/questions/61533331/clearing-pending-exti-interrupt-in-stm32f103.(accessed October 10, 2024).
```

## **Troubleshooting**

### **1. motor PWM duty ratio for different DIR**

When, DIR=0 duty=0.8--> PWM 0.8 // 실제 모터에 전달되는 pwm

Whe, DIR=1 duty=0.8--> PWM 0.2 // 실제 모터에 전달되는 PWM

SOL)

```
float targetPWM;  // pwm for motor input
float duty=abs(DIR-targetPWM); // duty with consideration of DIR=1 or 0

PWM_duty(PWM_PIN, duty);
```

### **2. Motor does not run under duty 0.5**

SOL) Configure motor PWM period as 1kHz

### **3. Check and give different Interrupt Priority**

SOL) Check if you have different NVIC priority number for each IRQs
