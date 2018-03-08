/*
  Strain_Gauge.cpp - Library for strain gauge.
  Created by Roman Kolbert, Frebruary 22, 2016.
*/



#include "Arduino.h"
#include "Strain_Gauge.h"
#include <SPI.h>   

Strain_Gauge::Strain_Gauge(int *listento_pointer, ADC_Chip *adc, int multiplex_channel, int digipot_channel, int input_pin, int virt_grnd_pin, int level, int select_slave_pin)
{
		_listento_pointer=listento_pointer;
		_adc = adc;
		_level = level;
        _multiplex_channel = multiplex_channel;
		_input_pin = input_pin;
		_virt_grnd_pin = virt_grnd_pin;
		_select_slave_pin = select_slave_pin;
		_digipot_channel = digipot_channel;
		pinMode(select_slave_pin, OUTPUT);
        pinMode(MULTIPLEXER_A, OUTPUT);
        pinMode(MULTIPLEXER_B, OUTPUT);
		_doCalibration = false;
		SPI.begin();
        digitalPotWrite(_digipot_channel, _level);
}

Strain_Gauge::~Strain_Gauge()
{
	
}

int Strain_Gauge::readStrain()
{
	if(!(*_listento_pointer == _multiplex_channel)) return _strain_gauge_input;
	_strain_gauge_input_raw = analogRead(_input_pin);
	
	//HACK because in actual version is no virtual ground input pin
	_virtual_ground = _adc->readADC(_virt_grnd_pin);
	_strain_gauge_input =  _strain_gauge_input_raw - _virtual_ground;

	return _strain_gauge_input;
	
}

boolean Strain_Gauge::calibrate()
{
	if (_strain_gauge_input > 2 && _level < 255) _level++;
	else if (_strain_gauge_input <-2 && _level > 0 )  _level--;
	else{ 
		_doCalibration = false;
		return true;
	}
	digitalPotWrite(_digipot_channel, _level);
    delay(10);
    return false;
}

void Strain_Gauge::digitalPotWrite(int address, int value) {
  // take the SS pin low to select the chip:
  digitalWrite(_select_slave_pin, LOW);

  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(_select_slave_pin, HIGH);
}

int Strain_Gauge::getLevel() {
	return _level;
}

void Strain_Gauge::setLevel(int level) {
	 digitalPotWrite(_digipot_channel, level);
	_level = level;
}

void Strain_Gauge::goCalibrate(){
	_doCalibration = true;
    //set_multiplex_channel();
}

void Strain_Gauge::listen_to_me(){
	*_listento_pointer = _multiplex_channel;
	set_multiplex_channel();
	delay(1000);
}

void Strain_Gauge::set_multiplex_channel(){
    switch(_multiplex_channel){
        case 0:
            digitalWrite(MULTIPLEXER_A, LOW);
            digitalWrite(MULTIPLEXER_B, LOW);
            break;
        case 1:
            digitalWrite(MULTIPLEXER_A, LOW);
            digitalWrite(MULTIPLEXER_B, HIGH);
            break;
        case 2:
            digitalWrite(MULTIPLEXER_A, HIGH);
            digitalWrite(MULTIPLEXER_B, LOW);
            break;
        case 3:
            digitalWrite(MULTIPLEXER_A, HIGH);
            digitalWrite(MULTIPLEXER_B, HIGH);
            break;
        }
}
