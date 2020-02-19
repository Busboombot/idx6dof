

// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#include <AccelStepper.h>
#include <Wire.h>

#define MAX_SPEED 10000

#define MAX_ACCEL 15000

#define ACCEL_TIME 10 // time in sec to target aceeelerating to close diff between current and target pos

bool position_is_initialized = false;

AccelStepper stepper( AccelStepper::DRIVER, 2,3);

void setup() {
  
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
  Serial.print("Starting");

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCEL);
  
}

void loop() {
  stepper.run();
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  
  char axis; 
  int a = 0;
  
  
  union {
    int32_t pos;
    byte b[sizeof(int32_t)];
  };

  axis = Wire.read();

  for(int i = 0; i < sizeof(int32_t); i++){
    b[i] = Wire.read(); 
  }
  
  //Serial.print(axis); 
  //Serial.print(' '); 
  //Serial.println(a);   
  //Serial.print(' '); 
  //Serial.println(stepper.currentPosition());         

  if (!position_is_initialized) {
    position_is_initialized = true;
    stepper.setCurrentPosition(pos);
  }
 
  stepper.moveTo(pos);

  //a = 2.0 * float(abs(pos - stepper.currentPosition()))/(ACCEL_TIME*ACCEL_TIME);
  
  //if (a > MAX_ACCEL){
  //  a = MAX_ACCEL;
  //}

  //stepper.setAcceleration(a);

  
}
