# **LAB: Stepper Motor**

**Date:** 2024-10-26

**Author/Partner:** Lee Chankeun

**Github:** [go to Github(Link)](https://github.com/lcg0070/Embedded_Controller/tree/main/LAB/LAB6_Stepper_Motor)

**Demo Video:** [go to youtube(Link)](https://youtu.be/-TFCnZ1kNlY)

## **Introduction**

This lab focuses on controlling a stepper motor using the GPIO outputs of a microcontroller, employing a Finite State Machine (FSM) to design the control algorithm.


### **Requirement**

### **Hardware**

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 3Stepper Motor 28BYJ-48
  - Motor Driver A4988 (tutorial)
  - Motor Driver ULN2003 (lab)
  - breadboard

### **Software**

- PlatformIO installed IDE, EC_HAL library

---



## **Problem : Stepper Motor with 4-input sequence**

For problem 1, will use stepper motor driver **ULN2003 motor driver** [See here for ULN2003 spec sheet](https://www.electronicoscaldas.com/datasheet/ULN2003A-PCB.pdf)).  
MCU will gives 4-input pulses in sequence.  
This report aims to determine the motor's maximum and minimum speeds by rapidly increasing and decreasing its speed to the fullest extent possible.

### **Hardware Connection**

Specification sheet of the motor and the motor driver for wiring and min/max input voltage/current.

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB6_Stepper_Motor/report/images/stepper_motor_datasheet.png?raw=true">  
<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB6_Stepper_Motor/report/images/motor_wiring_driver.png?raw=true">



### **Stepper Motor Sequence**

Unipolar stepper motor is used for this lab

**Full-stepping sequence**

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB6_Stepper_Motor/report/images/full_step_sequence.png?raw=true">

| Phase | Port_Pin  | Sequences |   |   |   |   |
|-------|-----------|-----------|---|---|---|---|
|       |           |           | 1 | 2 | 3 | 4 | 
| A     | PB_10     |           | H | L | L | H |
| B     | PB_4      |           | H | H | L | L |
| A'    | PB_5      |           | L | H | H | L |
| B'    | PB_3      |           | L | L | H | H |

Full-stepping Sequence


**Half-stepping sequence**

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB6_Stepper_Motor/report/images/half_step_sequence.png?raw=true">

| Phase | Port_Pin  | Sequences |   |   |   |   |   |   |   |   |
|-------|-----------|-----------|---|---|---|---|---|---|---|---|
|       |           |           | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
| A     | PB_10     |           | H | H | L | H | L | L | L | H |
| B     | PB_4      |           | L | H | H | L | L | L | L | L |
| A'    | PB_5      |           | L | L | L | L | H | H | L | L |
| B'    | PB_3      |           | L | L | L | H | L | H | H | H |

Half-stepping Sequence


### **Finite State Machine**

Draw a State Table for Full-Step Sequence. You can choose either Moore FSM or Mealy.

- Full-Stepping Sequence

| State | Next State |         | Output   |
|-------|------------|---------|----------| 
|       | DIR = 0    | DIR = 1 | (ABA'B') |
| s0    | S3         | S1      | HHLL     |
| s1    | S0         | S2      | LHHL     |
| s2    | S1         | S3      | LLHH     |
| s3    | S2         | S0      | HLLH     |



- Half-Stepping Sequence

| State | Next State |         | Output   |
|-------|------------|---------|----------| 
|       | DIR = 0    | DIR = 1 | (ABA'B') |
| s0    | S7         | S1      | HLLL     |
| s1    | S0         | S2      | HHLL     |
| s2    | S1         | S3      | LHLL     |
| s3    | S2         | S4      | HLLH     |
| s4    | S3         | S5      | LLHL     |
| s5    | S4         | S6      | LLHH     |
| s6    | S5         | S7      | LLLH     |
| s7    | S6         | S0      | HLLH     |


### **Configuration**

| Digital Out          | SysTick |
|----------------------|---------|
| PB10, PB4, PB5, PB3  | delay() |
| NO Pull-up Pull-down |         |
| Push-Pull            |         |
|  Fast                |         |

### **Discussion**

1. Find out the trapezoid-shape velocity profile for a stepper motor. When is this profile necessary?

   > A trapezoid-shape velocity profile is a motion control technique that ensures
   >    - smooth acceleration 
   >    - constant velocity
   >    - deceleration phases for stepper motors
   > 
   >    This profile minimizes mechanical stress and reduces the risk of step loss.    
   > When application requires high precision and stability, this profile is needed.

2. How would you change the code more efficiently for micro-stepping control? You don’t have to code this but need to explain your strategy.

   > 1. Timer interrupts will be used to precise pulse timing for microstepping
   >    - Since timer interrupts allow for precise timing of commands to the motor drive, interrupt-based approach instead of using the current method
   > 2. Adjust micro-step resolution dynamically
   >    - Higher micro stepping(half step) at low speeds provides better control, while lower micro stepping (full step) at high speeds reduces processing overhead
   


### **Code**

github link : [Go to github](https://github.com/lcg0070/Embedded_Controller/tree/main/LAB/LAB6_Stepper_Motor)

Variable and Function Descriptions   
```setup()``` Configures the system clock, GPIO pins, and initializes the stepper motor.   
```EXTI15_10_IRQHandler()``` Interrupt handler for the external interrupt on BUTTON_PIN. When triggered, it stops the stepper motor.

Initialization (main and setup)   
The main function first calls ```setup()``` for initial configuration.   
```Stepper_step(2048, 1, FULL)```  calls Stepper_step to move the motor 2048 steps in a specified direction with a defined mode (FULL or HALF).



Clock and Systick:
```RCC_PLL_init()``` Sets up the system clock.   
```SysTick_init()``` Initializes the SysTick timer for delays.


External Interrupt:

```EXTI_init(BUTTON_PIN, FALL, 0)``` Sets up an external interrupt on BUTTON_PIN (likely GPIOC pin 13) with a falling-edge trigger.
```GPIO_init(BUTTON_PIN, INPUT)``` Configures the pin as an input.   

Stepper Motor Setup:   
The ```PinName_t pinnames[4]``` array defines the pins used for the stepper motor control ```(PB_10, PB_4, PB_5, PB_3)```.   
```Stepper_init(pinnames)``` sets up the GPIO pins for the stepper motor.   
```Stepper_setSpeed(2)``` sets the motor speed.   
```EXTI15_10_IRQHandler``` This interrupt handler detects a button press on BUTTON_PIN. When triggered, it calls Stepper_stop() to halt the motor. The interrupt flag is then cleared with clear_pending_EXTI(BUTTON_PIN).

Copy

```c++
// /*----------------------------------------------------------------\
// Author           : Lee ChanKeun
// Created          : 10-22-2024
// Modified         : 10-22-2024
// Language/ver     : C in CLION with platformio
//
// Description      : LAB_Stepper_motor
// /----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecSTM32F4.h"


void setup(void);

int main(void) {
    // Initialiization --------------------------------------------------------
    setup();



    for(int i = 0; i < 10; i++) {
        Stepper_step(2048, 1, FULL);  // (Step : 2048, Direction : 0 or 1, Mode : FULL or HALF)
    }
    // Inifinite Loop ----------------------------------------------------------
    while(1) {

    }
}

// Initialiization
void setup(void){
    RCC_PLL_init();                                 // System Clock = 84MHz
    SysTick_init();                                 // Systick init

    EXTI_init(BUTTON_PIN, FALL,0);           // External Interrupt Setting
    GPIO_init(BUTTON_PIN, INPUT);           // GPIOC pin13 initialization

    PinName_t pinnames[4] = {PB_10, PB_4, PB_5, PB_3};
    Stepper_init(pinnames);              // Stepper GPIO pin initialization
    Stepper_setSpeed(2);                          	//  set stepper motor speed
}

void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN)) {
        Stepper_stop();
        while(1){;}
        clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
    }
}

```



### **Results**

Experiment images and results

> (<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB6_Stepper_Motor/report/images/stepper_motor_datasheet.png?raw=true" width=50% height=50%>)
>

 [demo video link](https://youtu.be/-TFCnZ1kNlY)

## **Reference**

Complete list of all references used (github, blog, paper, etc)

## **Troubleshooting**

