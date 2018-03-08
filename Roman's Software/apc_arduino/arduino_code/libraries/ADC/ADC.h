/*
  MCP3008.h - Library for communicating with MCP3008 Analog to digital converter.
  Created by Uros Petrevski, Nodesign.net 2013
  Released into the public domain.
  
  ported from Python code originaly written by Adafruit learning system for rPI :
  http://learn.adafruit.com/send-raspberry-pi-data-to-cosm/python-script
*/

#ifndef ADC_h
#define ADC_h

#include "Arduino.h"
#include "SPI.h"

class ADC_Chip
{
  public:
    ADC_Chip(int cspin);
    int readADC(int adcnum);
  private:
      int _cspin;
};


#endif
