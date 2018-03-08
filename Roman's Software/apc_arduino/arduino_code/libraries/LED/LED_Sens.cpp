/*
  LED_Sens.cpp - Library for LED Sensing.
  Created by Roman Kolbert, Frebruary 23, 2016.
*/


#include "Arduino.h"
#include "LED_Sens.h"

LED_Sens::LED_Sens( int input_pin)
{
		_input_pin = input_pin;
}

LED_Sens::~LED_Sens()
{
	
}

int LED_Sens::readLED()
{
	_LED_input = analogRead(_input_pin);
	return _LED_input;
	
}
