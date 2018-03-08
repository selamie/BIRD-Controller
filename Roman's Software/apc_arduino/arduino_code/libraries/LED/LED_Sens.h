/*
  LED_Sens.h - Library for LED Sensing.
  Created by Roman Kolbert, Frebruary 23, 2016.
*/

#ifndef LED_SENS_H
#define LED_SENS_H

#include "Arduino.h"

class LED_Sens
{
public:
	LED_Sens(int input_pin);
	~LED_Sens();
	int readLED();
private:
	int _input_pin; //The analog pin the LED is connected to
	int _LED_input;
};

#endif
