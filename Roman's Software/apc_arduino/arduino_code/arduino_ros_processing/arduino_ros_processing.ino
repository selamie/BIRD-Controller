/*
  arduino_ros_processing
  Main code for Arduino. Reduced to ros functions.
  Functions of actors and sensors are hidden in libraries.
  Created by Roman Kolbert, Frebruary 23, 2016.
*/


// Get Pins List 
#include <Pin_List.h>

// ROS Library
#define USE_USBCON // Only for Arduino Micro
#include <ros.h>


 
// Libraries for actors and sensors
#include <Strain_Gauge.h>
#include <LED_Sens.h>
#include <Spatula.h>
#include <Suction_Cup.h>


// Libraries for ROS Topics
#include <apc_arduino/StrainGaugeData.h>
#include <apc_arduino/LEDData.h>
#include <apc_arduino/Spatula_Position.h>
#include <apc_arduino/HallEffectData.h>
#include <apc_arduino/SuctionSensData.h>


// Libraries for ROS Services
#include <apc_arduino/MoveToPos.h>
#include <apc_arduino/SuctionCupSwitch.h>
#include <apc_arduino/TuneStrainGauge.h>
#include <apc_arduino/Calib_Spatula.h>
#include <apc_arduino/ServiceBridge.h>


// Namespaces for ROS Services
using apc_arduino::MoveToPos;
using apc_arduino::SuctionCupSwitch;
using apc_arduino::TuneStrainGauge;
using apc_arduino::Calib_Spatula;
using apc_arduino::ServiceBridge;


// Handle for ROS Node. Can be treated like in usual cpp code.
ros::NodeHandle nh;

// Initiation of ROS messages for ROS tpoics
apc_arduino::StrainGaugeData sg_msg;
apc_arduino::LEDData led_msg;
apc_arduino::Spatula_Position sp_pos_msg;
apc_arduino::HallEffectData hall_effect_msg;
apc_arduino::SuctionSensData suc_sens_msg;

// Initiation of ROS topics 
ros::Publisher strainGauges_pub("StrGData", &sg_msg);
ros::Publisher led_pub("LED_Data", &led_msg);
ros::Publisher sp_pub("Spatula_pos", &sp_pos_msg);
ros::Publisher hall_effect_pub("HEData", &hall_effect_msg);
ros::Publisher suc_sens_pub("SucSenData", &suc_sens_msg);

int i_am_listening_to = -1; 

// Initiation of actor/sensor objects out of the libraries
// ADC Chip 
#include <ADC.h>
ADC_Chip adc(ADC_CS);
Strain_Gauge straingauges[4] = { Strain_Gauge(&i_am_listening_to,&adc, 0,0), Strain_Gauge(&i_am_listening_to,&adc,1,1), Strain_Gauge(&i_am_listening_to,&adc,2,2), Strain_Gauge(&i_am_listening_to,&adc,3,3)};
LED_Sens photodiodes[4] = { LED_Sens(PHOTO_DIODE_0),LED_Sens(PHOTO_DIODE_1),LED_Sens(PHOTO_DIODE_2),LED_Sens(PHOTO_DIODE_3)};
Spatula spatulas[2] = { Spatula(MOTOR1_OPEN,MOTOR1_CLOSE,ENCODER_DECODER1_CS,1,1), Spatula(MOTOR2_OPEN,MOTOR2_CLOSE,ENCODER_DECODER2_CS,2,1)};
Suction_Cup cups[2] = {Suction_Cup(SUC_CUP1),Suction_Cup(SUC_CUP2)};



// Callbacks for ROS services 

void bridge_callback(const ServiceBridge::Request  & req, ServiceBridge::Response & res){
  switch(req.service){
    // Spatulas go to position
    case 0:
      if(0 < req.channel && req.channel < 3) spatulas[req.channel-1].go(req.value);  
      break;
    // Spatula Calibration
    case 1:
      switch(req.channel){
        case 1:
         spatulas[0].calib(req.value);
          break;
        case 2:
          spatulas[1].calib(req.value);
          break;
      }
      break;
    // Suction Cup On off
    case 2:
      if(0 < req.channel && req.channel < 3){
        if(req.value == 1) cups[req.channel -1].turn_on();
        else cups[req.channel -1].turn_off();
      } else if(req.channel == 3){
          if(req.value == 1){
              cups[0].turn_on();
              cups[1].turn_on();
          }else {
              cups[0].turn_off();
              cups[1].turn_off();
          }
        }
        break;
    // Strain Gauges 
    case 3:
      // Specific Strain Gauge
      if(req.channel > -1 && req.channel < 4){
          straingauges[req.channel].listen_to_me();
          //delay(1000);
          for (int max_count = 0; max_count < 255; max_count++){
            sg_msg.strain0 = straingauges[req.channel].readStrain();
            sg_msg.level0=   straingauges[req.channel].getLevel();
            strainGauges_pub.publish(&sg_msg); 
            if (straingauges[req.channel].calibrate()) break;
          }
      }
      // All Strain Gauges
      if(req.channel == 5){
        for(int sg = 0; sg < 4; sg++){
          straingauges[sg].listen_to_me();
          //delay(1000);
          for (int max_count = 0; max_count < 255; max_count++){
            sg_msg.strain0 = straingauges[sg].readStrain();
            sg_msg.level0=   straingauges[sg].getLevel();
            strainGauges_pub.publish(&sg_msg); 
            if (straingauges[sg].calibrate()) break;
          }
        }
      }
      break;
    // Listen to Strain Gauge
    case 4:
      if(req.channel > -1 && req.channel < 4){
          if (i_am_listening_to != req.channel) straingauges[req.channel].listen_to_me();
      }
  }
}


// Initiate ROS service
ros::ServiceServer<ServiceBridge::Request, ServiceBridge::Response> serviceBridge("Arduino_Services",&bridge_callback);


void setup() {
  // ROS Node Initialization
  nh.initNode();

  // Topics are published
  nh.advertise(strainGauges_pub);
  nh.advertise(led_pub);
  nh.advertise(sp_pub);
  nh.advertise(hall_effect_pub);
  nh.advertise(suc_sens_pub);
  // Advertise Service 
  nh.advertiseService(serviceBridge);
 
}

void loop() {
  // Read sensors and publish data

  //Strain Gauges
  sg_msg.strain0=  straingauges[0].readStrain();
  sg_msg.level0=   straingauges[0].getLevel();
  sg_msg.strain1=  straingauges[1].readStrain();
  sg_msg.level1=   straingauges[1].getLevel();
  sg_msg.strain2=  straingauges[2].readStrain();
  sg_msg.level2=   straingauges[2].getLevel();
  sg_msg.strain3=  straingauges[3].readStrain();
  sg_msg.level3=   straingauges[3].getLevel();

  strainGauges_pub.publish(&sg_msg);

  // LEDs
  led_msg.LED0= photodiodes[0].readLED();
  led_msg.LED1= photodiodes[1].readLED();
  led_msg.LED2= photodiodes[2].readLED();
  led_msg.LED3= photodiodes[3].readLED();

  led_pub.publish(&led_msg);

  //Spatula Position
  sp_pos_msg.Sp1 = spatulas[0].get_pos();
  sp_pos_msg.Sp2 = spatulas[1].get_pos();
  sp_pos_msg.Sp1_moving = spatulas[0].get_moving();
  sp_pos_msg.Sp2_moving = spatulas[1].get_moving();
  sp_pub.publish(&sp_pos_msg);

  //Hall Effect
  hall_effect_msg.HE0 = adc.readADC(HALL_EFFECT1_CHANNEL);
  hall_effect_msg.HE1 = adc.readADC(HALL_EFFECT2_CHANNEL);
  hall_effect_msg.HE2 = adc.readADC(HALL_EFFECT3_CHANNEL);
  hall_effect_msg.HE3 = adc.readADC(HALL_EFFECT4_CHANNEL);
  hall_effect_pub.publish(&hall_effect_msg);

  //Suction Cup Sensor
  suc_sens_msg.SucSen0 = adc.readADC(SUCTION_SENSOR1_CHANNEL);
  suc_sens_msg.SucSen1 = adc.readADC(SUCTION_SENSOR2_CHANNEL);
  suc_sens_pub.publish(&suc_sens_msg);
 
  spatulas[0].spinOnce();
  spatulas[1].spinOnce();
  nh.spinOnce();
  delay(20);
}
