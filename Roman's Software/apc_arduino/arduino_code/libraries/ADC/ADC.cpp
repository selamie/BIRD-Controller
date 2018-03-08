/*
  MCP3008.cpp - Library for communicating with MCP3008 Analog to digital converter.
  Created by Uros Petrevski, Nodesign.net 2013
  Released into the public domain.
  
  ported from Python code originaly written by Adafruit learning system for rPI :
  http://learn.adafruit.com/send-raspberry-pi-data-to-cosm/python-script
*/

#include "ADC.h"

ADC_Chip::ADC_Chip(int cspin) {
    
    // define SPI outputs and inputs for bitbanging
    
    SPI.begin();
    _cspin = cspin;

    
    pinMode(_cspin, OUTPUT);
	digitalWrite(_cspin, HIGH); // 
    
}

// read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
int ADC_Chip::readADC(int adcnum) {

  if ((adcnum > 7) || (adcnum < 0)) return -1; // Wrong adc address return -1

  digitalWrite(_cspin, LOW); //     # bring CS low

  int commandout = adcnum;
  commandout |= 0x18; //  # start bit + single-ended bit 0000 0001 | 0001 1000 ->  0001 1001 -> 11001 
  commandout <<= 4; //    # we only need to send 5 bits here
 
  SPI.transfer(0x01); // Start bit 
  uint8_t byte1 = SPI.transfer(commandout);
  uint8_t byte2 = SPI.transfer(0x00);
  
  int adcout = (int) (byte1 & B00000011); 
  adcout <<= 8;
   
  
 adcout |= (byte2);

  digitalWrite(_cspin, HIGH);

  return adcout;
}


