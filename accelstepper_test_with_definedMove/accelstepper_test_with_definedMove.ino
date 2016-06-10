
/*  Axis Test
A basic test of the operation of the IDXAxis class, using the pentant to drive all six axes. 
This code only runs on an Arduino Due with the IDX Teach Pendant attached. The pendant has currently 
has hard-coded pin assignements. 
*/
#include "idx_axis.h"
#include "idx_pendant.h"
#include <AccelStepper.h>

#define NUM_AXES 6
#define STEP_DWELL 40 // Delay, in microseconds, to dwell on the step pulses
#define UPDATE_DELAY 100 // time, in milliseconds, between velocity updates
#define ACCEL 700 // Acceleration for moving forward or back
#define STOP_ACCEL 1500 // Acceleration for stopping

IDXPendant pendant;

int axis_switches[] = {IDX_SW_AXIS0, IDX_SW_AXIS1, IDX_SW_AXIS2, IDX_SW_AXIS3, IDX_SW_AXIS4, IDX_SW_AXIS5 };

int target_velocity = 0; // Only one velocity switch, so it is the same for all axes
int switch_pos_velocities[3];

int8_t sw_state;
int8_t bounce;
#define SET_MOVE_STPS 500

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


void setup() {

  switch_pos_velocities[IDX_SW_POS_TOP] = 550;
  switch_pos_velocities[IDX_SW_POS_MID] = 250;
  switch_pos_velocities[IDX_SW_POS_BOTTOM] = 75;

  pendant.begin();

  for( int i = 0; i < NUM_AXES; i++){
    motors[i].setMaxSpeed(550);
    motors[i].setAcceleration(ACCEL);

  }


}

AccelStepper *motor;

void loop() {

  if (millis() > next_update){

      pendant.run_once();

      target_velocity = switch_pos_velocities[pendant.sw_pos(IDX_SW_SPEED)];
      sw_state = pendant.sw_pos(IDX_SW_STEP);
      
      for (int i = 0; i < NUM_AXES; i++){

        motor = motors + i;
        switch (sw_state) {
          case 0:
            switch (pendant.sw_pos(axis_switches[i])) {
              case 0:
                if (bounce == 0) {
                  motor->move(-1*SET_MOVE_STPS);
                  bounce = 1;
                }
              break;
              case 1:
                bounce = 0;
              break;
              case 2:
                if (bounce == 0) {
                  motor->move(SET_MOVE_STPS);
                  bounce = 1;
                }
              break;
            }
          break;
          case 1:
            switch(pendant.sw_pos(axis_switches[i])){
              case IDX_SW_POS_TOP:
                motor->setAcceleration(ACCEL);
                motor->moveTo(motor->currentPosition()+1000);
                break;
    
              case IDX_SW_POS_MID: 
                motor->setAcceleration(STOP_ACCEL);
                motor->moveTo(motor->currentPosition());
                break;
    
              case IDX_SW_POS_BOTTOM: 
                motor->setAcceleration(ACCEL);
                motor->moveTo(motor->currentPosition()-1000);
                break;
            }
          break;
        }
      }

      next_update = millis() + UPDATE_DELAY;
    }

  for( int i = 0; i < NUM_AXES; i++){
    motors[i].run();
  }
}

