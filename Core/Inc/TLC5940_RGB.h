/*
 * shrooms.h
 *
 *  Created on: Feb. 15, 2019
 *      Author: wierie
 */

#ifndef INC_TLC5940_RGB_H_
#define INC_TLC5940_RGB_H_

#include "TLC5940.h"

void set_led(int led, int r, int g, int b);
//int getpatternvalue(int pattern, double angle);

typedef struct{
	double r_angle;
	double g_angle;
	double b_angle;

	double r_steps;
	double g_steps;
	double b_steps;

	int	rval;
	int gval;
	int bval;
}Led;

typedef struct{
	int	ledindex;
	int channel;
	int32_t stepselect;
	int patternselect;
}Led_selection;



#endif /* INC_TLC5940_RGB_H_ */
