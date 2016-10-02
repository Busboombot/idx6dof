
#include <Arduino.h>
#include <limits.h>
#include "idx_command.h"
#include "DueTimer.h"

#include "CRC32.h"

#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )

#define N_AXES 6
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
  
  int dwell = 4; // in us

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  
  uint8_t active_axes = 0;

  uint8_t pins[N_AXES] = {2,3,4,5,6,7};

  uint32_t now = micros();
  uint32_t last_high_tick_time[N_AXES] = {now};
  uint32_t last_low_tick_time[N_AXES] = {now};
  uint8_t pin_high[N_AXES] = {false};
  
  uint8_t axis_pins[N_AXES] = {2,3,4,5,6,7};
  
  for (;;) {
    
    
    cbuf.run();

    if (active_axes == 0 && msg != 0){
      cbuf.sendResponse(*msg, IDX_COMMAND_DONE );
      //Serial.print("Done ");Serial.println(msg->seq);
      delete msg;
      msg = 0;
    }

    now = micros();
    if( cbuf.size() > 0 && msg == 0 ){
      msg = cbuf.getMessage();
      for (int axis = 0; axis < N_AXES; axis ++){
        last_high_tick_time[axis] = now;
      }
      //Serial.print("Axis ");Serial.print(msg->seq);Serial.print(" ");Serial.print(steps[0]);Serial.print(" ");Serial.println(ticks[0]);
    }

    active_axes = 0;
    for (int axis = 0; axis < N_AXES; axis ++){
      
      if (msg && msg->steps[axis] > 0 && ((unsigned long)(now - last_high_tick_time[axis])   > msg->ticks[axis])){ 
        fastSet(pins[axis]);
        last_high_tick_time[axis] +=  msg->ticks[axis];
        last_low_tick_time[axis] = now;
        pin_high[axis] = true;
      }

      if (pin_high[axis] && ((unsigned long)(now - last_low_tick_time[axis])  > STEP_DWELL)){ 
        fastClear(pins[axis]);
        msg->steps[axis]--;
        pin_high[axis] = false;
      }

      if ( msg->steps[axis] > 0){
        active_axes ++;
      }
    }
  }

  return 0;
}

