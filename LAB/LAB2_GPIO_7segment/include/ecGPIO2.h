/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Lee ChanKeun
Created          : 09-10-2024
Modified         : 08-23-2024
Language/ver     : C in CLION with platformio

Description      : GPIO functions
/----------------------------------------------------------------*/

#ifndef ECGPIO2_H
#define ECGPIO2_H

#include "stm32f411xe.h"
#include "ecRCC2.h"
#include "ecPinNames.h"

// MODER
#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

// OTYPER
#define OUTPUT_PUSH_PULL       0x00
#define OUTPUT_OPEN_DRAIN      0x01


// OSPEED
#define LOW_SPEED       0x00
#define MEDIUM_SPEED    0x01
#define FAST_SPEED      0x10
#define HIGH_SPEED      0x11

// PUPDR
#define NO_PULLUP_PULLDOWN  0x00
#define PULL_UP             0x01
#define PULL_DOWN           0x10
#define RESERVED            0x11

#define HIGH 1
#define LOW  0

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

// GPIO_Config
void GPIO_init(PinName_t pinName, uint32_t mode);
void GPIO_write(PinName_t pinName, int Output);
int  GPIO_read(PinName_t pinName);
void GPIO_mode(PinName_t pinName, uint32_t mode);
void GPIO_ospeed(PinName_t pinName, int speed);
void GPIO_otype(PinName_t pinName, int type);
void GPIO_pupd(PinName_t pinName, int pupd);

// seven_segment
void sevensegment_display_init(PinName_t pinNameA, PinName_t pinNameB, PinName_t pinNameC, PinName_t pinNameD);
void sevensegment_display(uint8_t  num);

void sevensegment_decoder_init(void);
void sevensegment_decoder(uint8_t  num);

#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif //ECGPIO2_H