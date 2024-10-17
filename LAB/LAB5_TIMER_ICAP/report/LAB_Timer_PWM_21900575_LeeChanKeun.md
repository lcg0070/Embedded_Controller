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

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F91526930%2F198864049-3dba8f8d-aec8-4f9a-8da3-9adc0fe0e4b9.png&width=768&dpr=4&quality=100&sign=7898ae7d&sv=1

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

The program needs to

- Generate a trigger pulse as PWM to the sensor.
- Receive echo pulses from the ultrasonic sensor
- Measure the distance by calculating pulse-width of the echo pulse.
- Display measured distance in [cm] on serial monitor of Tera-Term for

  (a) 10mm (b) 50mm (c) 100mm


### **Configuration**

| System Clock | PWM | Input Capture |
| --- | --- | --- |
| PLL (84MHz) | PA6 (TIM3_CH1) | PB6 (TIM4_CH1) |
|  | AF, Push-Pull,
No Pull-up Pull-down, Fast | AF, No Pull-up Pull-down |
|  | PWM period: 50msec
pulse width: 10usec | Counter Clock : 0.1MHz (10us)
TI4 -> IC1 (rising edge)
TI4 -> IC2 (falling edge) |

### **Circuit Diagram**

> You need to include the circuit diagram
>

image

https://ykkim.gitbook.io/~gitbook/image?url=https%3A%2F%2Fuser-images.githubusercontent.com%2F38373000%2F192134563-72f68b29-4127-42ac-b064-2eda95a9a52a.png&width=768&dpr=4&quality=100&sign=60f2bbed&sv=1

### **Discussion**

1. There can be an over-capture case, when a new capture interrupt occurs before reading the CCR value. When does it occur and how can you calculate the time span accurately between two captures?

> Answer discussion questions
>
1. In the tutorial, what is the accuracy when measuring the period of 1Hz square wave? Show your result.

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

**Example Code**

Copy

```
/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2023-10-31 by YKKIM
* @brief   Embedded Controller:  LAB - Timer Input Capture
*					 						- with Ultrasonic Distance Sensor
*
******************************************************************************
*/

#include "stm32f411xe.h"
#include "math.h"
#include "ecSTM32F4v2.h"

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
	if(is_UIF(TIM4)){                     // Update interrupt
		__________													// overflow count
		clear_UIF(TIM4);  							    // clear update interrupt flag
	}
	if(is_CCIF(TIM4, 1)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
		time1 = __________;									// Capture TimeStart
		clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag
	}
	else if(__________){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
		time2 = __________;									// Capture TimeEnd
		timeInterval = __________; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
		ovf_cnt = 0;                        // overflow reset
		clear_CCIF(TIM4,2);								  // clear capture/compare interrupt flag
	}
}

void setup(){

	RCC_PLL_init();
	SysTick_init();
	UART2_init();

// PWM configuration ---------------------------------------------------------------------
	__________;			// PA_6: Ultrasonic trig pulse
	PWM_period_us(TRIG, 50000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us

// Input Capture configuration -----------------------------------------------------------------------
	__________;    	// PB_6 as input caputre
 	ICAP_counter_us(ECHO, 10);   	// ICAP counter step time as 10us
	ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
	__________;  // TIM4_CH2 as IC2 , falling edge detect

}

```

### **Results**

Experiment images and results

> Show experiment images /results
>

Add [demo video link](https://github.com/ykkimhgu/course-doc/blob/master/course/lab/link/README.md)

## **Reference**

Complete list of all references used (github, blog, paper, etc)

Copy

```

```

## **Troubleshooting**

(Option) You can write Troubleshooting section