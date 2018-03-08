/*
  LED_Sens.h - Library for the suction cup.
  Created by Roman Kolbert, Frebruary 23, 2016.
*/

#ifndef SUC_CUP_H
#define SUC_CUP_H

#include "Arduino.h"

class Suction_Cup
{
public:
	Suction_Cup(int output_pin);
	~Suction_Cup();
	void turn_on();
	void turn_off();
private:
	int _output_pin; //The analog pin the op amp is connected to
};

#endif
