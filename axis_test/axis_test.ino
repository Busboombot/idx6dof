
/*  Axis Test
A basic test of the operation of the IDXAxis class, using the pentant to drive all six axes. 
This code only runs on an Arduino Due with the IDX Teach Pendant attached. The pendant has currently 
has hard-coded pin assignements. 
*/
#include "idx_axis.h"
#include "idx_pendant.h"

#define NUM_AXES 6
#define STEP_DWELL 40 // Delay, in microseconds, to dwell on the step pulses
#define UPDATE_DELAY 100 // time, in milliseconds, between velocity updates

IDXAxis axes[] = {IDXAxis(2,3), IDXAxis(4,5), IDXAxis(6,7), IDXAxis(8,9), IDXAxis(10,11), IDXAxis(12, 13)};

IDXPendant pendant;

int axis_switches[] = {IDX_SW_AXIS0, IDX_SW_AXIS1, IDX_SW_AXIS2, IDX_SW_AXIS3, IDX_SW_AXIS4, IDX_SW_AXIS5 };

int target_velocity = 0; // Only one velocity switch, so it is the same for all axes
int switch_pos_velocities[3];

// NOTE! These only work on the SAM3X, or possibly other ARM Chips, but certainly the Arduino DUE. 
#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )

void setup() {

  switch_pos_velocities[IDX_SW_POS_TOP] = 31;
  switch_pos_velocities[IDX_SW_POS_MID] = 16;
  switch_pos_velocities[IDX_SW_POS_BOTTOM] = 5;

  pendant.begin();

  for (int i = 0; i < NUM_AXES; i++){
    axes[i].begin();
    axes[i].setVelocity(0);
  }

  Serial.begin(9600);
}

int tick = 0;
int next_update = 0; // TIme, in milis(), for the next velocity update. 
int run_update = false;


void loop() {
  
  while (true){
    tick ++;

    run_update = (millis() > next_update);

    for (int i = 0; i < NUM_AXES; i++){
        axes[i].startTick(tick);
    }

    if (run_update){

      pendant.run_once();

      target_velocity = switch_pos_velocities[pendant.sw_pos(IDX_SW_SPEED)];

    } else {
      delayMicroseconds(STEP_DWELL);
    }

    
    for (int i = 0; i < NUM_AXES; i++){
        axes[i].endTick();
    }

    if (run_update){
      for (int i = 0; i < NUM_AXES; i++){
        
        switch(pendant.sw_pos(axis_switches[i])){
          case IDX_SW_POS_TOP:
            axes[i].setVelocity(target_velocity);
            break;

          case IDX_SW_POS_MID: 
            axes[i].setVelocity(0);
            break;

          case IDX_SW_POS_BOTTOM: 
            axes[i].setVelocity(-target_velocity);
            break;
        }
     
        axes[i].updateVelocity();
      }
      
      next_update = millis() + UPDATE_DELAY;
      
    } else {
      delayMicroseconds(STEP_DWELL);
    }

  }

}

