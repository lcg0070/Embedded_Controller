# **LAB: Stepper Motor**

**Date:** 2024-10-26

**Author/Partner:** Lee Chankeun

**Github:** [go to Github(Link)](https://github.com/lcg0070/Embedded_Controller/tree/main/LAB/LAB6_Stepper_Motor)

**Demo Video:** [go to youtube(Link)](https://www.youtube.com/watch?v=-YlXUAXmJpU)

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

For problem 1, will use stepper motor driver **ULN2003 motor driver** [See here for ULN2003 spec sheet](https://www.electronicoscaldas.com/datasheet/ULN2003A-PCB.pdf))

MCU will gives 4-input pulses in sequence.

### **Hardware Connection**

Specification sheet of the motor and the motor driver for wiring and min/max input voltage/current.

<img src="">



### **Stepper Motor Sequence**

We will use unipolar stepper motor for this lab

Fill in the blanks of each output data depending on the below sequence.

**Full-stepping sequence**

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F91526930%2F197428513-f9a23147-3448-4bed-bda2-c90325b8c143.png&width=768&dpr=4&quality=100&sign=41eb2ac6&sv=1

Full-stepping Sequence

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F91526930%2F197428973-13acab66-049e-4f1c-be5c-176f9f15288b.png&width=768&dpr=4&quality=100&sign=a4ffa690&sv=1

**Half-stepping sequence**

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F91526930%2F197429006-d552ab16-0bbf-4c52-bdce-a0f2bfe5f0d8.png&width=768&dpr=4&quality=100&sign=50caa85c&sv=1

Half-stepping Sequence

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F91526930%2F197429050-173ac610-fa59-427d-b0c0-1e85ac20fbb2.png&width=768&dpr=4&quality=100&sign=9386ceb2&sv=1

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

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F91526930%2F197429145-243b63ac-86c4-4641-a7e0-1eb2277c00f4.png&width=768&dpr=4&quality=100&sign=1e0600f3&sv=1

- Half-Stepping Sequence

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F91526930%2F197429166-01b4e4e1-1579-4124-acb8-551176b030ea.png&width=768&dpr=4&quality=100&sign=aa07e6d7&sv=1

You have to program the stepping sequence using the state table. You can define the states using structures.

Read [Tutorial: FSM programming for hints](https://ykkim.gitbook.io/ec/ec-course/lab/lab-smart-mini-fan-with-stm32-duino#example-code)

Copy

```
// State number
typedef enum StateNum {
	S0. S1, S2, S3
} StateNum;

typedef struct State {
	uint8_t out;
	StateNum next[2];
} State_t;

State_t FSM[4] = {
	{0x9 , {S1, S3}},
	{0xA , {S2, S0}},
	{0x6 , {S3, S1}},
	{0x5 , {S0, S2}}
};
```

### **Create HAL library**

Download files:

- [ecStepper_student.h, ecStepper_student.c](https://github.com/ykkimhgu/EC-student/blob/main/include/lib-student/)

Then, change the library files as ecStepper.h, ecStepper.c

Declare and define the following functions in your library.

You must update your header files located in the directory `EC \lib\`.

**ecStepper.h**

Copy

```
// Initialize with 4 pins
// ( A, B,  AN,  BN)
void Stepper_init(PinName_t A, PinName_t B,  PinName_t AN, PinName_t BN);

// whatSpeed [rev/min]
void Stepper_setSpeed(long whatSpeed);

// Run for n Steps
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode);

// Immediate Stop.
void Stepper_stop(void);
```

> Note that these are blocking stepper controllers. While the stepper is running, the MCU cannot process other polling commands. If you can, modify it to be the non-blocking controller.
>

> You can also create your own functions different from the given instructions.
>

### **Procedure**

1. Create a new project under the directory `\repos\EC\LAB\LAB_Stepper_Motor`
  - The project name is “**LAB_Stepper_Motor”.**
  - Create a new source file named as “**LAB_Stepper_Motor.c”**

    > You MUST write your name on the source file inside the comment section.
>
2. Include your updated library in `\repos\EC\lib\` to your project.
  - **ecGPIO.h, ecGPIO.c**
  - **ecRCC.h, ecRCC.c**
  - **ecEXTI.h, ecEXTI.c**
  - **ecSysTick.h**, **ecSysTick.c**
  - **ecStepper.h** **ecStepper.h**
3. Connect the MCU to the motor driver and the stepper motor.
4. Find out the number of steps required to rotate 1 revolution using Full-steppping.
5. Then, rotate the stepper motor 10 revolutions with 2 rpm. Measure if the motor rotates one revolution per second.
6. Repeat the above process in the opposite direction.
7. Increase and decrease the speed of the motor as fast as it can rotate to find the maximum and minimum speed of the motor.
8. Apply the half-stepping and repeat the above.

### **Configuration**

| Digital Out | SysTick |
| --- | --- |
| PB10, PB4, PB5, PB3
NO Pull-up Pull-down
Push-Pull
Fast | delay() |

### **Discussion**

1. Find out the trapezoid-shape velocity profile for a stepper motor. When is this profile necessary?

   > Answer discussion questions
>
2. How would you change the code more efficiently for micro-stepping control? You don’t have to code this but need to explain your strategy.

   > Answer discussion questions
>

### **Code**

Your code goes here: [ADD Code LINK such as github](https://github.com/ykkimhgu/EC-student/)

Explain your source code with necessary comments.

Copy

```
// YOUR MAIN CODE ONLY
// YOUR CODE
```

**Sample Code : Stepper Motor**

Copy

```
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecSysTick.h"
#include "ecStepper.h"

void setup(void);

int main(void) {
	// Initialiization --------------------------------------------------------
	setup();

	Stepper_step(2048, 1, FULL);  // (Step : 2048, Direction : 0 or 1, Mode : FULL or HALF)

	// Inifinite Loop ----------------------------------------------------------
	while(1){;}
}

// Initialiization
void setup(void){

	RCC_PLL_init();                                 // System Clock = 84MHz
	SysTick_init();                                 // Systick init

	EXTI_init(BUTTON_PIN, FALL,0);           // External Interrupt Setting
	GPIO_init(BUTTON_PIN, EC_DIN);           // GPIOC pin13 initialization

	Stepper_init(PB_10,PB_4,PB_5,PB_3); // Stepper GPIO pin initialization
	Stepper_setSpeed(2);                          	//  set stepper motor speed
}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		Stepper_stop();
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}

```

### **Results**

Experiment images and results

> Show experiment images /results
>

Add [demo video link](https://github.com/ykkimhgu/course-doc/blob/master/course/lab/link/README.md)

## **Reference**

Complete list of all references used (github, blog, paper, etc)

## **Troubleshooting**

(Option) You can write Troubleshooting section