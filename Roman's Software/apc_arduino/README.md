Package to control Arduino

To make Arduino work in Ubuntu the current user has to be added to dialout group

	sudo usermod -aG dialout mcube

To make the Arduino Micro work on Ubuntu 14.04 the modemmanager needs to be uninstalled.

	sudo apt-get purge modemmanager

The Arduino is going to be connected automatically, but a rule has to be added to the system: 

	sudo cp usb_rule/52-arduino-micro.rules /etc/udev/rules.d/

Install Arduino IDE: [https://www.arduino.cc/en/Guide/Linux](https://www.arduino.cc/en/Guide/Linux)

Setup Arduino IDE: Go to File -> Preferences -> Set the Sketchbook location to the "arduino_code" directory in the apc_arduino package

Now it should be possible to compile and upload code to the Arduino. The program is calles "arduino_ros_processing". 

Launch Arduino Node:

	roslaunch apc_arduino start_arduino_node.launch

Start Arduino Node:
	
	rosrun apc_arduino serial_node.py /dev/ttyACM0
	
Device path could be different. Easiest way to find out is to check in the Arduino IDE.

TOPICS

LED_Data: Provides Sensor output of the sensor LED  
StrGData: Provides sensor output and tuning level of the strain gauge
HEData: Provides sensor output of Hall Effect Sensors
Spatula: Provides Spatula Position and "Moving" bool 

SERVICES

There is just one service with 3 parameters: service, channel and value.

service 0: Spatulas(channel 1 or 2). Go To position value.

service 1: Spatula(channel 1 or 2) Calibration. 

service 2: Suction cup (channel 1 or 2). Turn on(value=1) or off(value=0)

service 3: Tune Strain Gauge(channel 1,2,3,4 or 5 for all of them).

service 4: Listen to Strain Gauge(channel 1,2,3,4)
    
