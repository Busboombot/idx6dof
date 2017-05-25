
#include <Arduino.h>
#include <limits.h>
#include "idx_command.h"
#include "idx_stepper.h"
#include "bithacks.h"

#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )


/*
 * Initialize the board, serial ports, etc. 
 */
void init_board(){
  watchdogSetup();
  init(); // Initialize the board, probably
  delay(1);
  USBDevice.attach(); // Initialize the SerialUSB device. 

  Serial.begin(115200); // For debugging
  SerialUSB.begin(1050000); // For ros messages

  // Diagnostics
  pinMode(14, OUTPUT); // Loop tick
  pinMode(15, OUTPUT); // message tick
  
}

int main(void) {
  
  struct command * msg = 0;
  uint32_t now;
  uint8_t active_axes = 0;

  init_board();
 
  IDXCommandBuffer cbuf(SerialUSB);

  IDXStepper steppers[N_AXES] = {
    IDXStepper(0,2,3), 
    IDXStepper(1,4,5), 
    IDXStepper(2,6,7), 
    IDXStepper(3,8,9), 
    IDXStepper(4,10,11), 
    IDXStepper(5,12,13)
  };

  Serial.print("Command size :");Serial.println(sizeof(struct command));
  Serial.print("Response size:");Serial.println(sizeof(struct response));
  
  for (;;) {

    cbuf.startLoop(); // Start diagnostic times. 

    // Load a byte from the serial port and possibly process
    // add a completed message to the queue. 
    cbuf.run(); 
   
    /*
     * All of the axes have finished so clear out the message 
     */
    
    if (active_axes == 0 && msg != 0){
      //Serial.print("Clear message ");Serial.println(msg->seq);
      cbuf.sendDone(*msg);
      cbuf.resetLoopTimes();
      delete msg;
      msg = 0;
    }
    
    /*
     * If we have messages in the queue, and there is no message in progress, 
     * get the message and start working on it. 
     */

    if( cbuf.size() > 0 && msg == 0 ){
     
      msg = cbuf.getMessage();
      if (false){
        Serial.print("Start: ql="); Serial.print(cbuf.size()); 
        Serial.print(" Mesg#"); Serial.print(msg->seq); 
        Serial.print(" t="); Serial.print(msg->segment_time);
        Serial.print(" code="); Serial.print(msg->code);
        Serial.print(" v0="); Serial.print(msg->v0[0]);
        Serial.print(" v1="); Serial.print(msg->v1[0]);
        Serial.print(" x="); Serial.print(msg->steps[0]);
        Serial.print(" crc=");Serial.print(msg->crc); 
        Serial.println(" ");
      }
      
      for (int axis = 0; axis < N_AXES; axis ++){
        steppers[axis].setParams(micros(), msg->segment_time, msg->v0[axis], msg->v1[axis], msg->steps[axis]);
      }
    }
    
     /* Clear all of the pins, so setting a pin actually results in a
     * transition
     * 
     */
    
    for (int axis = 0; axis < N_AXES; axis ++){
      steppers[axis].clearStep();
    }
    
    /*
     * Iterate over all of the axes and step them when their time comes up. 
     */
-
    active_axes = 0;
    now = micros();
    for (int axis = 0; axis < N_AXES; axis ++){
      if(steppers[axis].step(now)){
         active_axes ++;
      }
    }

    cbuf.endLoop();

  }

  return 0;
}

