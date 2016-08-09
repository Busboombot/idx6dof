
/*  Axis Test
A basic test of the operation of the IDXAxis class, using the pentant to drive all six axes. 
This code only runs on an Arduino Due with the IDX Teach Pendant attached. The pendant has currently 
has hard-coded pin assignements. 
*/
#include "idx_axis.h"
#include "idx_pendant.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <limits.h>

#define NUM_AXES 6
#define STEP_DWELL 40 // Delay, in microseconds, to dwell on the step pulses
#define UPDATE_DELAY 100 // time, in milliseconds, between velocity updates
#define ACCEL 1000 // Acceleration for moving forward or back
#define STOP_ACCEL 2000 // Acceleration for stopping
#define MAX_SPEED 700 // 800 For the stepper test board


// NOTE! These only work on the SAM3X, or possibly other ARM Chips, but certainly the Arduino DUE. 
#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )

int tick = 0;
int next_update = 0; // TIme, in milis(), for the next velocity update. 
bool run_update = false;

AccelStepper motors[] = {
  AccelStepper(AccelStepper::DRIVER, 2, 3), AccelStepper(AccelStepper::DRIVER, 4, 5),
  AccelStepper(AccelStepper::DRIVER, 6, 7), AccelStepper(AccelStepper::DRIVER, 8, 9), 
  AccelStepper(AccelStepper::DRIVER, 10, 11), AccelStepper(AccelStepper::DRIVER, 12, 13)
};

struct command {
  char command_code;
  long params[10] = {LONG_MIN,LONG_MIN,LONG_MIN,LONG_MIN,LONG_MIN,LONG_MIN,LONG_MIN};
  bool command_complete;
};

struct command cmd;
int read_state = 0;
char buf[20] = {0};
int param_n = 0;
int buf_i = 0;

void serialEvent() {

  char c;

  while (Serial.available()) {
    
    c = Serial.read();
    Serial.print(c);

    if (isalpha(c)){ // command code
      cmd.command_code  = c;
      cmd.command_complete = false;
      param_n = 0;
      buf_i = 0;

    } else if( c == ' ' ){ // parameter separation
      if (buf_i > 0){
        buf[buf_i] = '\0';
        cmd.params[param_n] = atoi(buf);
        buf_i = 0;
        param_n++;
      }

    } else if (c == '\n'){ // End of command
      if (buf_i > 0){
        buf[buf_i] = '\0';
        cmd.params[param_n] = atoi(buf);
        buf_i = 0;
        param_n = 0;
      }
      cmd.command_complete = true;
      
      return;
     
    } else{ // A parameter
      buf[buf_i++] = c;
    }
  }
}

AccelStepper *motor;
MultiStepper multi;
float velocities[NUM_AXES];

void setup() {
  Serial.begin(115200);
    
  for( int i = 0; i < NUM_AXES; i++){
    motors[i].setMaxSpeed(MAX_SPEED);
    motors[i].setAcceleration(ACCEL);

    multi.addStepper(motors[i]);
    
  }

}



void loop() {

  if (cmd.command_complete){

    switch(cmd.command_code){       

      case 's': {
        for (int i = 0; i < NUM_AXES; i++){
          motors[i].setAcceleration(STOP_ACCEL);
          motors[i].moveTo(motor[i].currentPosition());
          motors[i].setMaxSpeed(0);
        }
        break;
      }

      case 'p':{
        Serial.print("p ");
        for (int i = 0; i < NUM_AXES; i++){
          if (i != 0){
            Serial.print(" ");
          }
          Serial.print(motors[i].currentPosition(), DEC); 
        }
        Serial.println("");
        break;
      }

      case 'm': {

        // Speed is set as a % of the configured max 0-100
        float v_max = (float)MAX_SPEED * ((float)cmd.params[0] /100.0);
        float x_max = 0; // Maximum distance traveled
        
        for (int i = 0; i < NUM_AXES; i++){
          if(cmd.params[i+1] != LONG_MIN ){
            x_max = max(x_max, abs(cmd.params[i+1]-motors[i].currentPosition()));
          }
          
        }

        float t = ((float)x_max) / v_max;
        
        for (int i = 0; i < NUM_AXES; i++){
          int param = cmd.params[i+1];
          if(param != LONG_MIN ){
            float x = (float)param -  motors[i].currentPosition();
            float v = abs(x) / t;
            
            motors[i].move(x);
            motors[i].setMaxSpeed(v);
            
            Serial.print("Move "); 
            Serial.print(i, DEC); Serial.print(": "); Serial.print(x, 6); Serial.print(" @ "); Serial.println(v, 6);
          }
        }
        break;
      }

    }
    cmd.command_complete = false;
    
    for( int i = 0; i < NUM_AXES; i++){
      cmd.params[i+1] = LONG_MIN;
    }
  }

  if (millis() > next_update){
    next_update = millis() + UPDATE_DELAY;
  }

  for( int i = 0; i < NUM_AXES; i++){
      motors[i].run();
  }
  
}

