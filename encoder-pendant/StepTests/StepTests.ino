

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

#define MAX_ACCEL 5000

#define ACCEL_TIME 10 // time in sec to target aceeelerating to close diff between current and target pos

bool position_is_initialized = false;

AccelStepper stepper( AccelStepper::DRIVER, 2,3);

void setup() {

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCEL);
}

int i = 0;

void loop() {


  if(!stepper.run()){

    switch(i%4){
      case 0:
        stepper.moveTo(0);
        break;
      case 1:
        stepper.moveTo(1000);
        break;
      case 2:
        stepper.moveTo(2000);
        break;
      case 3:
        stepper.moveTo(3000);
        break;
    }
    i++;

    
  }
  
}
