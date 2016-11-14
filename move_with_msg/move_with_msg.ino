
#include <Arduino.h>
#include <limits.h>
#include "idx_command.h"
#include "idx_stepper.h"
#include "bithacks.h"

#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )

void toggle(int pin){

  if (digitalRead(pin)){
    fastClear(pin);
  } else {
    fastSet(pin);
  }
  
}

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

  IDXStepper steppers[] = {
    IDXStepper(2,3), 
    IDXStepper(4,5), 
    IDXStepper(6,7), 
    IDXStepper(8,9), 
    IDXStepper(10,11), 
    IDXStepper(12,13), 
  };


  Serial.print("Starting. message size:");Serial.println(sizeof(struct command));
  
  for (;;) {

    toggle(14);

    cbuf.startLoop(); // Start diagnostic times. 

    // Load a byte from the serial port and possibly process
    // add a completed message to the queue. 
    cbuf.run(); 

    /*
     * All of the axes have finished so clear out the message 
     */
    if (active_axes == 0 && msg != 0){
      cbuf.sendDone(*msg);
      cbuf.resetLoopTimes();
      //cbuf.setPositions(positions);

      delete msg;
      msg = 0;
    }

    /*
     * If we have messages in the queue, and there is no message in progress, 
     * get the message and start working on it. 
     */
    
    if( cbuf.size() > 0 && msg == 0 ){
      toggle(15);
      msg = cbuf.getMessage();
  
      Serial.print(msg->code); Serial.print(" ");
      Serial.print(msg->seq); Serial.print(" ");
      Serial.print(msg->crc); Serial.println(" ");
      
      
      for (int axis = 0; axis < N_AXES; axis ++){
        steppers[axis].setParams(msg->n[axis], msg->cn[axis], msg->stepLeft[axis]);
      }
    }

    /*
     * Clear all of the pins, so setting a pin actually results in a
     * transition
     */
    for (int axis = 0; axis < N_AXES; axis ++){
      steppers[axis].clearStep();
    }

    /*
     * Iterate over all of the axes and step them when their time comes up. 
     */
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

