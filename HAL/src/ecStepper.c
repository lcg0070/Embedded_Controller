//
// Created by 이찬근 on 2024. 10. 22..
//

#include "stm32f4xx.h"
#include "ecStepper.h"

//State number
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7

#define PIN_NUMS 4

// Stepper Motor function
uint32_t direction = 1;
uint32_t step_delay = 100;
uint32_t step_per_rev = 64*32;


// Stepper Motor variable
volatile Stepper_t myStepper;


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {
 	{0b1100,{S1,S3}},		// ABA'B'
	{0b0110,{S2,S0}},
	{0b0011,{S3,S1}},
	{0b1001,{S0,S2}}
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = {	// 1000 , 1010 , 0010 , 0110 , 0100 , 0101, 0001, 1001
 	{0b1000,{S1,S7}},
 	{0b1100,{S2,S0}},
 	{0b0100,{S3,S1}},
 	{0b0110,{S4,S2}},
 	{0b0010,{S5,S3}},
 	{0b0011,{S6,S4}},
 	{0b0001,{S7,S5}},
 	{0b1001,{S0,S6}},
};


void Stepper_init(PinName_t pinNames[]){

	for(int i = 0; i < PIN_NUMS ; i++) {
		myStepper.pins[i] = pinNames[i];
		GPIO_init(pinNames[i], OUTPUT);
		GPIO_ospeed(pinNames[i], FAST_SPEED);
		GPIO_pupd(pinNames[i], NO_PULLUP_PULLDOWN);
		GPIO_otype(pinNames[i], OUTPUT_PUSH_PULL);
	}
}


void Stepper_pinOut (uint32_t state, uint32_t mode){

   	if (mode == FULL){         // FULL mode
   		for(int i = 0; i < PIN_NUMS; i++) {
   			GPIO_write(myStepper.pins[i], FSM_full[state].out >> i & 1UL);
   		}
	}
 	else if (mode == HALF){    // HALF mode
 		for(int i = 0; i < PIN_NUMS; i++) {
 			GPIO_write(myStepper.pins[i], FSM_half[state].out >> i & 1UL);
 		}
	}

}


void Stepper_setSpeed (long whatSpeed){      // rpm [rev/min]
	// 60 rpm = step_per_rev 1cycle
		step_delay = 60. * 1000. /(whatSpeed * (long)step_per_rev) ;	// Convert rpm to  [msec/step] delay
}


void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode){
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	 for(; myStepper._step_num > 0; myStepper._step_num--){				// run for step size
		delay_ms(step_delay);
	    if (mode == FULL)
	    	state = FSM_full[state].next[direction];

		else if (mode == HALF)
			state = FSM_half[state].next[direction];

		Stepper_pinOut(state, mode);

   	}
}


void Stepper_stop (void){
	myStepper._step_num = 0;
	// All pins(A,AN,B,BN) set as DigitalOut '0'
	for(int i = 0; i < PIN_NUMS; i++) {
		GPIO_write(myStepper.pins[i], 0UL);
	}
}