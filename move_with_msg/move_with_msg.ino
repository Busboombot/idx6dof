
#include <Arduino.h>
#include <limits.h>
#include "idx_command.h"
#include "bithacks.h"

#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )

#define STEP_DWELL 4

int main(void) {
  struct command * msg = 0;
  
  watchdogSetup();
  init(); // Initialize the board, probably
  delay(1);
  USBDevice.attach(); // Initialize the SerialUSB device. 

  //UART->UART_BRGR = 5; // 84000000 / 16 * x = BaudRate (write x into UART_BRGR)  5 -> 1050000
  Serial.begin(115200); // For debugging
  SerialUSB.begin(1050000); // For ros messages
 
  IDXCommandBuffer cbuf(SerialUSB);

  Serial.print("Starting. message size:");Serial.println(sizeof(struct command));
  
  int dwell = 4; // in us. Min dwell; actual is longer. 
  
  uint8_t active_axes = 0;

  uint8_t step_pins[N_AXES] = {2,4,6,8,10,12};
  uint8_t dir_pins[N_AXES] =  {3,5,7,9,11,13};

  int32_t positions[N_AXES] = {0}; // Axis positions in steps
  int32_t velocities[N_AXES] = {0}; // Axis velocities in steps / sec
  int32_t intervals[N_AXES] = {0}; // Inter-step interval in microseconds
  float accelerations[N_AXES] = {0}; // Accelerations in steps per sec^s
  
  uint32_t now = micros();
  uint32_t last_time[N_AXES] = {now};

  for(int i = 0; i < N_AXES; i++){
    pinMode(step_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
  }

  uint8_t work_step = 0;
  
  for (;;) {

    cbuf.startLoop();

    switch (work_step++ & 0x03 ){
      case 0: {

        break;
      }
      case 1: {

        break;
      }
      case 2: {

        break;
      }
      case 3: {

        break;
      }
    }
    
    cbuf.run();
    
    if (active_axes == 0 && msg != 0){
      cbuf.sendDone(*msg);
      cbuf.resetLoopTimes();
      cbuf.setPositions(positions);

      // The calculated velocities should be close to the target velocities, 
      // but if they are off a bit, this will bring them back to what was commanded. 
      for (int i = 0; i < N_AXES; i++){
        velocities[i] = msg->velocities[i];
      }
      
      delete msg;
      msg = 0;
    }

    now = micros();
    if( cbuf.size() > 0 && msg == 0 ){
      msg = cbuf.getMessage();
      for (int axis = 0; axis < N_AXES; axis ++){
        
        last_time[axis] = now;

        accelerations[axis] = float(velocities[axis] -  msg->velocities[axis])
        
      }
    }

    for (int axis = 0; axis < N_AXES; axis ++){
      fastClear(step_pins[axis]);
    }

    active_axes = 0;
    for (int axis = 0; axis < N_AXES; axis ++){
      
      if (msg && msg->steps[axis] > 0 && ((unsigned long)(now - last_time[axis])   > msg->ticks[axis])){ 
        
        if (velocities[axis] > 0){
          fastSet(dir_pins[axis]);
          positions[axis]++;
        } else {
          fastClear(dir_pins[axis]);
          positions[axis]--;
        }
        
        last_time[axis] +=  msg->ticks[axis];
        msg->steps[axis]--;
        fastSet(step_pins[axis]);

        velocities[axis] += accelerations[axis]*msg->ticks[axis]
        
      }

      if ( msg->steps[axis] > 0){
        active_axes ++;
      }
    }
    
    cbuf.endLoop();

  }

  return 0;
}

