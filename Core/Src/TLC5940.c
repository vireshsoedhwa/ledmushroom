/*
 * TLC5940.c
 *
 *  Created on: Feb 10, 2018
 *      Author: wierie
 */

#include "TLC5940.h"


//uint8_t dcData[12 * TLC5940_N] = {
//		0b11111111,		//12
//		0b11111111,		//11
//		0b11111111,		//10
//		0b11111111,		//9
//		0b11111111,		//8
//		0b11111111,		//7
//		0b11111111,		//6
//		0b11111111,		//5
//		0b11111111,		//4
//		0b11111111,		//3
//		0b11111111,		//2
//		0b11111111,		//1
//};
//
//uint8_t gsData[24 * TLC5940_N] = {
//
//		0b00000000,
//		0b00000000,
//		0b00000000,
//		0b00000000,
//		0b00000000,		//5
//		0b00000001,
//		0b00000000,
//		0b00100000,
//		0b00000100,
//		0b00000000,		//10
//		0b10000000,
//		0b00010000,
//		0b00000010,
//		0b00000000,
//		0b01000000,		//15
//		0b00001000,
//		0b00000001,
//		0b00000000,
//		0b00100000,
//		0b00000100,		//20
//		0b00000000,
//		0b10000000,
//		0b00001111,
//		0b11111111,		//24
//};


void TLC5940_Gpio_SetHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
}

void TLC5940_Gpio_SetLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
}

void TLC5940_Pulse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
}
void TLC5940_Init(void)
{
	TLC5940_Gpio_SetLow(DCPRG_GPIO_Port,DCPRG_Pin);
	TLC5940_Gpio_SetHigh(VPRG_GPIO_Port,VPRG_Pin);
	TLC5940_Gpio_SetLow(XLAT_GPIO_Port,XLAT_Pin);
	TLC5940_Gpio_SetHigh(BLANK_GPIO_Port,BLANK_Pin);
}

void TLC5940_ClockInDC(void) {
	TLC5940_Gpio_SetHigh(DCPRG_GPIO_Port, DCPRG_Pin);
	TLC5940_Gpio_SetHigh(VPRG_GPIO_Port, VPRG_Pin);
	HAL_SPI_Transmit(&hspi1, dcData, dcDataSize, 1000);
	TLC5940_Pulse(XLAT_GPIO_Port,XLAT_Pin);
}

void TLC5940_SetGSandGS_PWM_IT(void){
	static uint8_t xlatNeedsPulse = 0;

	TLC5940_Gpio_SetHigh(BLANK_GPIO_Port,BLANK_Pin);

	if (HAL_GPIO_ReadPin(VPRG_GPIO_Port, VPRG_Pin) == GPIO_PIN_SET)
	{
		TLC5940_Gpio_SetLow(VPRG_GPIO_Port, VPRG_Pin);
		if(xlatNeedsPulse){
			TLC5940_Pulse(XLAT_GPIO_Port, XLAT_Pin);
			xlatNeedsPulse = 0;
		}
	}else if (xlatNeedsPulse){
			TLC5940_Pulse(XLAT_GPIO_Port,XLAT_Pin);
			xlatNeedsPulse = 0;
		}

	TLC5940_Gpio_SetLow(BLANK_GPIO_Port, BLANK_Pin);
	// 4096 cycles after this

	if (gsUpdateFlag) {
		HAL_SPI_Transmit(&hspi1, gsData, gsDataSize, 1000);
		xlatNeedsPulse = 1;
		gsUpdateFlag = 0;
	}
}


void TLC5940_SetAllGS(uint16_t value) {
	uint8_t tmp1 = (value >> 4);
	uint8_t tmp2 = (uint8_t)(value << 4) | (tmp1 >> 4);
	gsData_t i = 0;
	do {
	gsData[i++] = tmp1;
	gsData[i++] = tmp2;
	gsData[i++] = (uint8_t)value;
	}
	while (i < gsDataSize);
}

void TLC5940_SetAllDC(uint8_t value) {

	uint8_t tmp1 = (uint8_t)(value << 2);
	uint8_t tmp2 = (uint8_t)(tmp1 << 2);
	uint8_t tmp3 = (uint8_t)(tmp2 << 2);
	tmp1 |= (value >> 4);
	tmp2 |= (value >> 2);
	tmp3 |= value;
	dcData_t i = 0;

	do {
		dcData[i++] = tmp1;
		dcData[i++] = tmp2;
		dcData[i++] = tmp3;
	} while (i < dcDataSize);
}

void TLC5940_SetGS(channel_t channel, uint16_t value) {
	channel = numChannels - 1 - channel;
	uint16_t i = (uint16_t)channel * 3 / 2;
	switch (channel % 2) {
		case 0:
			gsData[i] = (value >> 4);
			i++;
			gsData[i] = (gsData[i] & 0x0F) | (uint8_t)(value << 4);
			break;
		default: // case 1:
			gsData[i] = (gsData[i] & 0xF0) | (value >> 8);
			i++;
			gsData[i] = (uint8_t)value;
			break;
	}
}

void TLC5940_SetDC(channel_t channel, uint8_t value) {
	channel = numChannels - 1 - channel;
	uint16_t i = (uint16_t)channel * 3 / 4;
	switch (channel % 4) {
		case 0:
			dcData[i] = (dcData[i] & 0x03) | (uint8_t)(value << 2);
			break;
		case 1:
			dcData[i] = (dcData[i] & 0xFC) | (value >> 4);
			i++;
			dcData[i] = (dcData[i] & 0x0F) | (uint8_t)(value << 4);
			break;
		case 2:
			dcData[i] = (dcData[i] & 0xF0) | (value >> 2);
			i++;
			dcData[i] = (dcData[i] & 0x3F) | (uint8_t)(value << 6);
			break;
		default: // case 3:
			dcData[i] = (dcData[i] & 0xC0) | (value);
			break;
	}
}

