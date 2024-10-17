# **LAB: Input Capture - Ultrasonic**

**Author/Partner:** Lee Chankeun

**Github:** [go to Github(Link)](https://github.com/lcg0070/Embedded_Controller/tree/main/LAB/LAB4_PWM)

**Demo Video:** [go to youtube(Link)](https://youtu.be/7i9oOtYHT6E)

## **Introduction**

This experiment focuses on the development of a microcontroller-based system for distance measurement using an ultrasonic sensor. 
The primary objectives of this lab are:  
  - To create a program that interfaces with an ultrasonic distance sensor
  - To implement input capture mode for precise distance measurements  
  - To utilize timer output functionality for generating sensor trigger pulses

### **Requirement**

### **Hardware**

- MCU
    - NUCLEO-F411RE
- Actuator/Sensor/Others:
    - HC-SR04
    - breadboard

### **Software**

- PlatformIO installed IDE, EC_HAL library

## **Problem 1: Create HAL library**

Library for "Input capture" was made for this experiment

**ecCAP2.h**

```
/**
 *ICAP_pinmap : initialize channel and pin according to the input pinName
 *  Parameter :
 *      -PinName_t   : input PinName
 *      -TIM_TypeDef : input TIM register
 *      -chN         : input channel number
**/
void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);

 /**
  *ICAP_init : initialize the GPIO, TIM, TIMER register value
  *  Parameter :
  *      -PinName_t   : input PinName
 **/
void ICAP_init(PinName_t pinName);

 /**
 *ICAP_setup : initialize ICAP value(edge type, IC number etc.)
 *  Parameter :
 *      -PinName_t   : input PinName
 *      -ICn         : input channel number
 *      -edge_type   : input edge_type
**/
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);

/**
 *ICAP_counter_us : set the timer value to 1[usec] when pll is 84[Mhz]
 *  Parameter :
 *      -PinName_t   : input PinName
 *      -usec        : input PSC value
**/
void ICAP_counter_us(PinName_t pinName, int usec);


 /**
 *ICAP_capture : capture the ICAP value
 *  Parameter :
 *      -TIMx        : Pin_number's Tim register's value
 *      -ICn         : Input Channel number
**/
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn);


/**
*is_CCIF : Check, if it is CCIF true of not
*  Parameter :
*      -TIMx        : Pin_number's Tim register's value
*      -CCnum       : Timx's CCR register value
**/
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);  // CCnum= 1~4


/**
*clear_CCIF : clear CCIF value
*  Parameter :
*      -TIMx        : Pin_number's Tim register's value
*      -CCnum       : Timx's CCR register value
**/
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);
```

## **Problem 2: Ultrasonic Distance Sensor (HC-SR04)**

The HC-SR04 ultrasonic distance sensor.  
This economical sensor provides 2cm to 400cm of non-contact measurement functionality with a ranging accuracy that can reach up to 3mm.   
Each HC-SR04 module includes an ultrasonic transmitter, a receiver and a control circuit.

HC-SR04

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB5_TIMER_ICAP/report/images/img.png?raw=true">

**The HC-SR04 Ultrasonic Range Sensor Features:**

- Input Voltage: 5V
- Current Draw: 20mA (Max)
- Digital Output: 5V
- Digital Output: 0V (Low)
- Sensing Angle: 30° Cone
- Angle of Effect: 15° Cone
- Ultrasonic Frequency: 40kHz
- Range: 2cm - 400cm


3. Connect the HC-SR04 ultrasonic distance sensor to MCU pins(PA6 - trigger, PB6 - echo), VCC and GND

### **Measurement of Distance**

This program will measure with this sequence

- Generate a trigger pulse as PWM to the sensor.
- Receive echo pulses from the ultrasonic sensor
- Measure the distance by calculating pulse-width of the echo pulse.
- Display measured distance in [cm] on serial monitor of Tera-Term for

  (a) 10mm (b) 50mm (c) 100mm


### **Configuration**

| System Clock | PWM                        | Input Capture                 |
|--------------|----------------------------|-------------------------------|
| PLL (84MHz)  | PA6 (TIM3_CH1)             | PB6 (TIM4_CH1)                |
|              | AF, Push-Pull,             |                               |
|              | No Pull-up Pull-down, Fast | AF, No Pull-up Pull-down      |
|              | PWM period: 50msec         | Counter Clock : 0.1MHz (10us) |                         |
|              | pulse width: 10usec        | TI4 -> IC1 (rising edge)      |
|              |                            | TI4 -> IC2 (falling edge)     |


### **Circuit Diagram**

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB5_TIMER_ICAP/report/images/diagram.png?raw=true" width=50% height=50%>

### **Discussion**

1. There can be an over-capture case, when a new capture interrupt occurs before reading the CCR value. When does it occur and how can you calculate the time span accurately between two captures?

> There could be an over-capture case by   
>    1. high capture rates  
>    2. processing delay 
>    3. limited counter/timer range   
> 
> By applying the formula ```time_span = end_capture_time - start_capture_time + overflow_count*timer_max_value```.  
> Can calculate the time span accurately between two captures
>
1. In the tutorial, what is the accuracy when measuring the period of 1Hz square wave? Show your result.

> Sensor operates at 40kHz. But when the 1Hz is applied the sensor can't get the value every sampling cycle.  
> So the detected value had a lot of error or a value of zero was output.  
> 

### **Code**

Your code goes here: [ADD Code LINK such as github](https://github.com/lcg0070/Embedded_Controller/tree/main/LAB/LAB5_TIMER_ICAP)

Explain your source code with necessary comments.

```c++


// /*----------------------------------------------------------------\
// Author           : Lee ChanKeun
// Created          : 10-17-2024
// Modified         : 10-17-2024
// Language/ver     : C in CLION with platformio
//
// Description      : LAB_TIMER_ICAP
// /----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "math.h"
#include "ecSTM32F4.h"


// variable declare
uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

#define TRIG PA_6
#define ECHO PB_6

void setup(void);

int main(void){

	setup();

	while(1){
		distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
		printf("%f cm\r\n", distance);
		delay_ms(500);
	}
}


void TIM4_IRQHandler(void){
	// Check for Update interrupt
	if(is_UIF(TIM4)){															  // Update interrupt
		ovf_cnt++; 																  // overflow count
		clear_UIF(TIM4);  														  // clear update interrupt flag
	}
	// Check for TIM4_Ch1 (IC1) Capture Flag (Rising Edge)
	if(is_CCIF(TIM4, 1)){ 												  // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = ICAP_capture(TIM4, 1);										  // Capture TimeStart
		clear_CCIF(TIM4, 1);												  // clear capture/compare interrupt flag
	}
	// Check for TIM4_Ch2 (IC2) Capture Flag (Falling Edge)
	else if(is_CCIF(TIM4, 2)){ 											  // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2 = ICAP_capture(TIM4, 2);										  // Capture TimeEnd
		timeInterval = (time2 - time1 + (float)(ovf_cnt * (TIM4->ARR+1))) * 0.01; // (10us * counter pulse -> [msec] unit) Total time of echo pulse
		ovf_cnt = 0;															  // overflow reset
		clear_CCIF(TIM4,2);												  // clear capture/compare interrupt flag
	}
}


void setup(){

	RCC_PLL_init();
	SysTick_init();
	UART2_init();

	// PWM configuration ---------------------------------------------------------------------
	PWM_init(PA_6, U_SEC, 1);			// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);					// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);		// PWM pulse width of 10us

	// Input Capture configuration -----------------------------------------------------------------------
	ICAP_init(PB_6);    						// PB_6 as input caputre
	ICAP_counter_us(ECHO, 10);   			// ICAP counter step time as 10us
	ICAP_setup(ECHO, 1, IC_RISE);   // TIM4_CH1 as IC1 , rising edge detect
	ICAP_setup(ECHO, 2, IC_FALL);   // TIM4_CH2 as IC2 , falling edge detect
}
```

### **Results**

Experiment images and results

<img src="https://github.com/lcg0070/Embedded_Controller/blob/main/LAB/LAB4_PWM/report/images/dc_result.png?raw=true" width=50% height=50%>


[demo video link](https://www.youtube.com/watch?v=-YlXUAXmJpU)

## **Reference**

Complete list of all references used (github, blog, paper, etc)

Copy

```

```

## **Troubleshooting**

(Option) You can write Troubleshooting section