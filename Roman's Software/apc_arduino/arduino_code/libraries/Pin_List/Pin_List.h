/*
  pin_list.h - This header defines all the DIO and Analog Input Pins on the Arduino Micro.
  Created by Roman Kolbert, May 20, 2016.
*/

#ifndef PIN_LIST_H
#define PIN_LIST_H

///////////////// DIGITAL ////////////////////
// Suction Cups
#define SUC_CUP1 3
#define SUC_CUP2 2

// MOTOR for Finger 1
#define MOTOR1_OPEN 9
#define MOTOR1_CLOSE 8
#define ENCODER_DECODER1_CS 13

// MOTOR for Finger 2
#define MOTOR2_OPEN 7
#define MOTOR2_CLOSE 10
#define ENCODER_DECODER2_CS 12


// Multiplexer for Strain Gauge Circuit
#define MULTIPLEXER_A 4
#define MULTIPLEXER_B 5

// Digipot
#define DIGIPOT_CS 6



///////////////// ANALOG /////////////////////

// Strain Gauges Analog Input
#define STRAIN_GAUGE_INPUT 4

//Photo Diodes
#define PHOTO_DIODE_0 0
#define PHOTO_DIODE_1 1
#define PHOTO_DIODE_2 2
#define PHOTO_DIODE_3 3

//////////////// Analog Digital Converter Chip ////////////////

#define ADC_CS 11
#define HALL_EFFECT1_CHANNEL 0
#define HALL_EFFECT2_CHANNEL 1
#define HALL_EFFECT3_CHANNEL 4
#define HALL_EFFECT4_CHANNEL 5

#define SUCTION_SENSOR1_CHANNEL 3
#define SUCTION_SENSOR2_CHANNEL 7

#define VIRTUAL_GROUND 6


#endif
