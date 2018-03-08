/*
  Spatula.h - Library for controlling the motor of a spatula.
  Created by Roman Kolbert, Frebruary 22, 2016.
*/

#ifndef SPAT_H
#define SPAT_H

#include "Arduino.h"


class Spatula
{
public:
	Spatula(int open_pin, int close_pin, int slave_select_pin, int spatula_no, int encoder_direction);
	~Spatula();
	boolean go(int goal_position);
	void spinOnce();
	void stop();
	long get_pos();
	void calib(int calib_val);
    boolean get_moving();
	
private:
	int _encoder_direction;
	void _initEncoders();
	long _readEncoder();
	void _clearEncoderCount();
    void _writeEncoderCount(long input);
	int _spatula_no;
	int _open_pin;
	int _close_pin;
	boolean _encoder0PinALast;
	long _position;
	long _goal_position;
	long _last_position;
	int _slave_select_pin;
	int _init_position;
	int _same_position_counter;	
	boolean _opening;
	boolean _closing;
    long _readEEPROM();
    void _writeEEPROM(long input);
	int _encoder_pin_a;
	void _zero();
	boolean _docalib;
    int _calib_val;

};

#endif
