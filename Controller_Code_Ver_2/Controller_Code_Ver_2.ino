/*author: Selam Gano
 * This is an impedance controller
 * Lever attached to the motor should stay at specified position theta_d, in radians
 * 
 * need to install DualVNH5019MotorShield library if you haven't yet. 
 * 
 * 
 * motor driver documentation: https://github.com/pololu/dual-vnh5019-motor-shield
 */

// Inclde the standard Arduino SPI Library, please ensure the SPI pins are
// connected properly for your Arduino version
#include <SPI.h>

// Slave Select pin
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
 
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc1,HIGH);
  
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation 

}
long readEncoder() {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder 1
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
 
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation   
  
}

//**********PROGRAM START*****************

//set your desired angle:
const float theta_d = 2*3.14159;

//pins
const int motorA = 2;
const int motorB = 3;
const int motorPWM = 9; //threshold to move: analogWrite command of ~35
const int current1 = A0;

//properties taken from motor data sheet
//
const float R = 5.3667; //winding resistance, ohms //found by stalling motor measuring current & knowing voltage
const float kb = 0.000147; //back emf, mV/min --> 0.884mV/min * 1V/1000mV *1min/60s = V/s //STILL FROM FAULHABER MOTOR
const float ki = 3.8989; //current constant //3.8989 A/nm found by stalling motor & measuring current divide by min stall torque
const float pulses = 739.908; //184.977; 


//set gains
float k = 0.6; //torque control position gain
float b = 0.02; //torque control velocity gain
float k_p = 0; //proportional gain for current

//for use in PID controller
float pid_k_p = 0.1;
float pid_k_d = 0.05;
float pid_k_i = 0;

void setup() {
  
  Serial.begin(9600);
  initEncoders();       Serial.println("Encoders Initialized...");  
  clearEncoderCount();  Serial.println("Encoders Cleared...");
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(current1, INPUT);
  
}


//********************LOOP*****************

void loop() {
  
  
  float v = calculateVelocity();
  command(v);
  float pos = getAngle();
  Serial.println("pos:");
  Serial.println(pos);
  Serial.println("v:");
  Serial.println(v); 
    
} 


//*************FUNCTIONS*******************

float calculateVelocity(){
  //expects integer indicating encoder
  //returns impedance controlled velocity (in m/s?)

//get parameters
float theta_k = getAngle(); //rad 
float w = getVelocity(); //rad/s
float i_k = getCurrent(); //A

//calculations
float theta = theta_k - theta_d;

float tau = k*theta + b*w; //control law
float i_d = tau/ki; //desired current from torque control/torque-to-current constant

float v = R*i_d + k_p*(i_d - i_k) + kb*w;  //commanded voltage //with k_p = 0, v = 5.36*i_d + 0.884*w

return v; 
}

float calculatePIDvelocity(){
  //expects integer indicating encoder
  //returns impedance controlled velocity
                float error_count = 0;
                float time_step = 0;
                float e_old = 0;

//get parameters
float theta_k = getAngle(); //rad
float w = getVelocity(); //rad/s
float i_k = getCurrent(); //A?

//calculations
float e_k = theta_d - theta_k;
float e_dotk = 0 - ((e_k - e_old)/0.001);
error_count = error_count + e_k;
time_step += 0.001;
e_old = e_k;

float v = pid_k_p *e_k + pid_k_d*e_dotk + pid_k_i*(error_count/time_step);  //commanded voltage

return v; 
  
}

void command(float v){
    int state;
    if (v < 0) {
      digitalWrite(motorA, 1);
      digitalWrite(motorB, 0);
      state = -1;
      } 
    else {
      digitalWrite(motorA, 0);
      digitalWrite(motorB, 1);
      state = 1;
      }
    float volts2pwm = 255/12.0;
    float v_go = abs(v);

    int commanded;
    if (v_go >= 12){
      commanded = 255;
      analogWrite(motorPWM, commanded);
      
    }
    else{
      commanded = round(volts2pwm*v_go);    
      analogWrite(motorPWM, commanded);
    }
    
}


float getCurrent() {
  float raw = analogRead(current1);
  float i = ((5.0/1024.0)*raw);
  return i;
}

float getVelocity() {
  //expects input denoting which encoder to read
  //outputs velocity with units of rad/s 
  float t1 = millis(); 
  float angle1 = getAngle();
  float t2 = millis();
  float angle2 = getAngle();
  
  while (t2<=(t1+1)){
    t2 = millis();
    angle2 = getAngle();
  }

  float dt = (t2-t1)/1000.0; //convert to seconds
  float velocity = (angle2-angle1)/dt;
  
  return velocity;
}

float getAngle() { 
  //expects integer denoting which encoder to read (motor 1 or 2)
  //outputs the current shaft angle in radians

  float revs = readEncoder()/pulses;
  float rad = revs*2*3.14159;
  return rad;
}



void fwd(float speed) {
  //Expects speed input between 0 and 255
  //speed must be between 0 and 90 for clockwise
  
  digitalWrite(motorA, 1);
  digitalWrite(motorB, 0);
  analogWrite(motorPWM, speed);
}

void back(int speed) { 
  
  digitalWrite(motorA, 0);
  digitalWrite(motorB, 1);
  analogWrite(motorPWM, speed);
}

void halt(){
  digitalWrite(motorA, 0);
  digitalWrite(motorB, 0);
}

