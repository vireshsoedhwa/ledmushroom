/*
 * shrooms.c
 *
 *  Created on: Feb. 15, 2019
 *      Author: wierie
 */

#include <TLC5940_RGB.h>
#include "math.h"

/**
 *
 * @param led - the led to control
 * @param r	- red channel
 * @param g - green channel
 * @param b - blue channel
 */

void set_led(int led, int r, int g, int b)
{
	led = led + 1; // zero indexing from 0 to 4
	int bchannel = 2 * led + ( led - 2);
	int gchannel = bchannel + 1;
	int rchannel = bchannel + 2;

	TLC5940_SetGS(bchannel, b);
	TLC5940_SetGS(gchannel, g);
	TLC5940_SetGS(rchannel, r);
}

//int getpatternvalue(int pattern, double angle)
//{
//	int thevalue = 0;
//
//	switch (pattern)
//	{
//	case 0:
//		thevalue = (int)( (0.5*(1+sin(angle))) * 4095);
//		break;
//	case 1:
//		thevalue = (int)( (0.5*(1+sin(sin(angle)))) * 4095);
//		break;
//	default:
//		thevalue = (int)( (0.5*(1+sin(angle))) * 4095);
//		break;
//	}
//
//	return thevalue;
//
//}

