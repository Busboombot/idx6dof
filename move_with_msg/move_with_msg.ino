
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
  int32_t positions[N_AXES] = {0};
  
  uint32_t now = micros();
  uint32_t last_high_tick_time[N_AXES] = {now};

  for(int i = 0; i < N_AXES; i++){
    pinMode(step_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
  }
  
  for (;;) {

    cbuf.startLoop();
    cbuf.run();
    
    if (active_axes == 0 && msg != 0){
      cbuf.sendDone(*msg);
      cbuf.resetLoopTimes();
      cbuf.setPositions(positions);
      delete msg;
      msg = 0;
    }

    now = micros();
    if( cbuf.size() > 0 && msg == 0 ){
      msg = cbuf.getMessage();
      for (int axis = 0; axis < N_AXES; axis ++){
        last_high_tick_time[axis] = now;
      }
    }

    for (int axis = 0; axis < N_AXES; axis ++){
      fastClear(step_pins[axis]);
    }

    active_axes = 0;
    for (int axis = 0; axis < N_AXES; axis ++){
      
      if (msg && msg->steps[axis] > 0 && ((unsigned long)(now - last_high_tick_time[axis])   > msg->ticks[axis])){ 
        if (B_IS_SET(msg->directions, axis)){
          fastSet(dir_pins[axis]);
          positions[axis]++;
        } else {
          fastClear(dir_pins[axis]);
          positions[axis]--;
        }
        last_high_tick_time[axis] +=  msg->ticks[axis];
        msg->steps[axis]--;
        fastSet(step_pins[axis]);

      }

      if ( msg->steps[axis] > 0){
        active_axes ++;
      }
    }
    cbuf.endLoop();
  }

  return 0;
}

