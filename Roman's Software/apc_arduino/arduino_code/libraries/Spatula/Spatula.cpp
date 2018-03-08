/*
  Strain_Gauge.cpp - Library for controlling the motor of a spatula.
  Created by Roman Kolbert, Frebruary 22, 2016.
*/




#include "Spatula.h"

#include <SPI.h>


#include "Arduino.h"

#include <EEPROM.h>

#include "digitalWriteFast.h"

Spatula::Spatula(int open_pin, int close_pin, int slave_select_pin, int spatula_no, int encoder_direction)
{
	_spatula_no = spatula_no;
	_open_pin = open_pin;
	_close_pin = close_pin;
	_slave_select_pin = slave_select_pin;
	_encoder_direction = encoder_direction;
	

	pinMode(_open_pin, OUTPUT);
	pinMode(_close_pin, OUTPUT);
	pinMode(_slave_select_pin, OUTPUT);

	
	digitalWrite(_open_pin, HIGH); //OPEN
	digitalWrite(_close_pin, HIGH); //CLOSE
	digitalWrite(_slave_select_pin, HIGH); //CLOSE
	
	_initEncoders();
    _position = _readEEPROM();
	_writeEncoderCount(_position);
	_last_position = _position;
	_opening = false;
	_closing = false;
	_same_position_counter = 0;
	_docalib = false;
    _calib_val = 2600 * _encoder_direction;
}

Spatula::~Spatula()
{
	
}

long Spatula::_readEEPROM(){
    long output;
    unsigned int output_1, output_2, output_3, output_4;
    
    output_1=EEPROM.read(0+_spatula_no*4);
    output_2=EEPROM.read(1+_spatula_no*4);
    output_3=EEPROM.read(2+_spatula_no*4);
    output_4=EEPROM.read(3+_spatula_no*4);
    
    output = (output_1 << 8) + output_2;
    output = (output << 8) + output_3;
    output = (output << 8) + output_4;
    
    return (long)output;
}

void Spatula::_writeEEPROM(long input){
    EEPROM.write(0+4*_spatula_no,(byte) ((unsigned long)input >> 24));
    EEPROM.write(1+4*_spatula_no,(byte) ((unsigned long)input >> 16));
    EEPROM.write(2+4*_spatula_no,(byte) ((unsigned long)input >> 8));
    EEPROM.write(3+4*_spatula_no,(byte) (unsigned long)input);
}

void Spatula::_zero(){
    long zero = _calib_val;
    _writeEEPROM(zero);
    _writeEncoderCount(zero);
}

boolean Spatula::get_moving(){
    return _opening || _closing;
}


void Spatula::calib(int calib_val){
    _calib_val = calib_val*_encoder_direction;
    _docalib = true;
    go(10000);
}

void Spatula::spinOnce(){

	
_position =  _readEncoder();
	
if(_opening || _closing){
    
			if(_opening && _goal_position > _position){
				digitalWrite(_open_pin, HIGH);
                //delay(1000);
				//_opening = false;
                //_writeEEPROM(_readEncoder());
			}
			if(_closing && _goal_position < _position){
				digitalWrite(_close_pin, HIGH);
                //delay(1000);
				//_closing = false;
                //_writeEEPROM(_readEncoder());
			} 
			if( _position < _last_position + 2 &&  _position > _last_position -2){
				 _same_position_counter++;
				 if(_same_position_counter > 20){
					 _opening = false;
					 _closing = false;
					 digitalWrite(_open_pin, HIGH);
					 digitalWrite(_close_pin, HIGH);
					_writeEEPROM(_readEncoder());
				 }
			 } else _same_position_counter=0;
	}else if(_docalib){
         _zero();
         //go(0); Changed after meeting today June 6 2016
         _docalib = false;
     }
_last_position = _position;
}

long Spatula::get_pos(){
	return _position*_encoder_direction;//(int) (0.01 * (double)_position);
}

boolean Spatula::go(int goal_position){
	
	_goal_position = (long) (goal_position * _encoder_direction);
	 	 
	if(_goal_position > _position){
		digitalWrite(_close_pin, LOW);
		_closing = true; 
	}

	if(_goal_position < _position){
		digitalWrite(_open_pin, LOW);
		_opening = true; 
	}
	
	return true;
	
}

void Spatula::stop(){
	_goal_position = _position;
}

void Spatula::_initEncoders() {

  SPI.begin();
  
  // Initialize encoder 
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  // digitalWrite(_slave_select_pin,LOW);        // Begin SPI conversation
  // SPI.transfer(0x88);                       // Write to MDR0
  // SPI.transfer(0x03);                       // Configure to 4 byte mode
  // digitalWrite(_slave_select_pin,HIGH);       // Terminate SPI conversation 

}
long Spatula::_readEncoder() {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
 
  // Read encoder 

    digitalWrite(_slave_select_pin,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(_slave_select_pin,HIGH);     // Terminate SPI conversation 

  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  // Hack for wrong wiring in Spatula 1
  //if(_spatula_no == 1) count_value=count_value * (-1);
  return count_value;
}

void Spatula::_writeEncoderCount(long input) {
    
  // Set encoder1's data register to 0
  digitalWrite(_slave_select_pin,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  // input = (1 << 24) + (2 << 16) + (3 << 8) + 4;
  SPI.transfer((byte) ((unsigned long)input >> 24));  // Highest order byte
  SPI.transfer((byte) ((unsigned long)input >> 16));           
  SPI.transfer((byte) ((unsigned long)input >> 8));           
  SPI.transfer((byte) (unsigned long)input);  // lowest order byte
  // SPI.transfer((byte) 4);  // Highest order byte
  // SPI.transfer((byte) 3);           
  // SPI.transfer((byte) 2);           
  // SPI.transfer((byte) 1);  // lowest order byte
  digitalWrite(_slave_select_pin,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(_slave_select_pin,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(_slave_select_pin,HIGH);     // Terminate SPI conversation   
}

void Spatula::_clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(_slave_select_pin,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(_slave_select_pin,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(_slave_select_pin,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(_slave_select_pin,HIGH);     // Terminate SPI conversation   
}

