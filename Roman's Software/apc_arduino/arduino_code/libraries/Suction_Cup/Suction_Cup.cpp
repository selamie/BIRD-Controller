/*
  LED_Sens.cpp - Library for the suction cup.
  Created by Roman Kolbert, Frebruary 23, 2016.
*/


#include "Arduino.h"
#include "Suction_Cup.h"

Suction_Cup::Suction_Cup(int output_pin)
{
		_output_pin = output_pin;
		pinMode(output_pin, OUTPUT);
		digitalWrite(_output_pin, LOW);	

}

Suction_Cup::~Suction_Cup()
{
	
}

void Suction_Cup::turn_on()
{
	digitalWrite(_output_pin, HIGH);	
}

void Suction_Cup::turn_off()
{
	digitalWrite(_output_pin, LOW);	
}
