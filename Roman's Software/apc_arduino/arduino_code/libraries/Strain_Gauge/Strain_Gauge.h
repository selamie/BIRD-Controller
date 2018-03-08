/*
  Strain_Gauge.h - Library for strain gauge.
  Created by Roman Kolbert, Frebruary 22, 2016.
*/

#ifndef STRAIN_GAUGE_H
#define STRAIN_GAUGE_H

#include "Arduino.h"
#include <Pin_List.h>
#include <ADC.h>

class Strain_Gauge
{
public:
	Strain_Gauge(int *listento_pointer, ADC_Chip *adc, int multiplex_channel, int digipot_channel,int input_pin = STRAIN_GAUGE_INPUT, int virt_grnd_pin = VIRTUAL_GROUND, int level = 160,  int select_slave_pin = DIGIPOT_CS);
	~Strain_Gauge();
	int readStrain();
	void goCalibrate();
	int getLevel();
	void setLevel(int level);
	boolean calibrate();
	void listen_to_me();
private:
	int * _listento_pointer;
	ADC_Chip *_adc; 
	int _input_pin; //The analog pin the op amp is connected to
	int _virt_grnd_pin; //The analog pin the virtual ground is connected to
	int _select_slave_pin; // the digital output pin the digi pot array (the chip) is connected to
	int _strain_gauge_input_raw;
	int _virtual_ground;
	int _strain_gauge_input;
	int _level; // the level of the digipot 
	int _digipot_channel; //The digi pot channel the strain gauge is connected to
    int _multiplex_channel;
	boolean  _doCalibration;
	void digitalPotWrite(int address, int value);
	
    void set_multiplex_channel();

};

#endif
