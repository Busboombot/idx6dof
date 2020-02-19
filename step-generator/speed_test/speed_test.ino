
//
// Requires
//  idxlib, https://github.com/Busboombot/idxlib

// RAMPS 1.4 pins
#define X_STEP_PIN         54
#define X_DIR_PIN          55


#include <Arduino.h>
#include <limits.h>
#include "fastset.h"


#define DEBUG_PRINT_ENABLED false
#define DEBUG_TICK_ENABLED false
#include "debug.h"

#define QUADRATURE_OUTPUT false

// N_AXES, defined in idx_command.h, is 6, 
// but this program can be defined to not iterate over all of them.  
//#define AXES_USED N_AXES
#define AXES_USED 1

#include "idx_command.h"
#include "idx_stepper.h"
#include "idx_quadgen.h"
#include "bithacks.h"


bool starvedToggle = false;

/*
 * Initialize the board, serial ports, etc. 
 */
void init_board(){
  watchdogSetup();
  init(); // Initialize the board, probably

  Serial.begin(115200); // For debugging
  Serial.println("Starting");
  init_debug();

}


int main(void) {
  
  
  uint32_t now;
  uint8_t active_axes = 0;
  int lastTime = 0;

  uint32_t segment_time = 1000000;
  int32_t v0 = 20000;
  int32_t v1= 20000;
  long steps = (v0 + v1)/2 * ( (float)segment_time / 1000000.0) ;

  init_board();
  
  IDXStepGenerator *stepper = new IDXStepper(0,X_STEP_PIN,X_DIR_PIN);




  for (;;) {
  
    if (active_axes == 0){
      fastDebugSet(PARAMS_TICK_PIN);
      stepper->setParams(micros(),segment_time, v0, v1, steps);
      fastDebugClear(PARAMS_TICK_PIN);
    }
  
    fastDebugSet(LOOP_STEP_TICK_PIN); 
    active_axes = 0;
    if(stepper->stepMaybe(micros())){
         active_axes ++; 
    }
    fastDebugClear(LOOP_STEP_TICK_PIN);

  }

  return 0;
}
