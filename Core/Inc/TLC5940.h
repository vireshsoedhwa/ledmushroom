/*
 * TLC5940.h
 *
 *  Created on: Feb 10, 2018
 *      Author: wierie
 */

#ifndef TLC5940_H_
#define TLC5940_H_



#endif /* TLC5940_H_ */

#include "gpio.h"
#include "spi.h"

#ifndef TLC5940_N
#define TLC5940_N 1
#endif

#if (12* TLC5940_N > 255)
#define dcData_t uint16_t
#else
#define dcData_t uint8_t
#endif

#if (24 * TLC5940_N > 255)
#define gsData_t uint16_t
#else
#define gsData_t uint8_t
#endif

#if (16 * TLC5940_N > 255)
#define channel_t uint16_t
#else
#define channel_t uint8_t
#endif
#define numChannels ((channel_t)16 * TLC5940_N)

#define dcDataSize ((dcData_t)12 * TLC5940_N)
#define gsDataSize ((gsData_t)24 * TLC5940_N)

uint8_t dcData[dcDataSize];
uint8_t gsData[gsDataSize];
volatile uint8_t gsUpdateFlag;

void TLC5940_Init(void);
void TLC5940_Gpio_SetHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void TLC5940_Gpio_SetLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void TLC5940_Pulse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void TLC5940_ClockInDC(void);
void TLC5940_SetGSandGS_PWM_IT(void);
void TLC5940_SetGS(channel_t channel, uint16_t value);
void TLC5940_SetAllGS(uint16_t value);
void TLC5940_SetAllDC(uint8_t value);
