
//
// Requires
//  idxlib, https://github.com/Busboombot/idxlib

#include <Arduino.h>
#include <limits.h>
#include "fastset.h"

#define DEBUG_PRINT_ENABLED true
#define DEBUG_TICK_ENABLED true

#include "debug.h"

#include "idx_command.h"
#include "idx_stepper.h"
#include "idx_quadgen.h"
#include "bithacks.h"


/*
 * Initialize the board, serial ports, etc. 
 */
void init_board(){
  watchdogSetup();
  init(); // Initialize the board, probably
  delay(1);
  USBDevice.attach(); // Initialize the SerialUSB device. 

  Serial.begin(115200); // For debugging
  SerialUSB.begin(115200); // For control messages. Max speed 1050000

  init_debug();

}


int main(void) {
  
  struct segment * segment = 0;
  uint32_t now;
  uint8_t active_axes = 0;
  int last_time = 0;
  bool starvedToggle = false;

  init_board();
 
  #if(DEBUG_PRINT_ENABLED)
  Serial.println("Starting");
  #endif
  
  IDXCommandBuffer cbuf(SerialUSB);

  for (;;) {

    // When idle, with no ticks being set ( but all are still cleared ) 
    // takes about 16.5us
    

    cbuf.run(); 

    if ( segment != 0){
      // Response, without setPositions: 42.5 us

      cbuf.sendDone(segment->seq);
      #if(DEBUG_PRINT_ENABLED)
        Serial.print("DONE: seq="); Serial.println(segment->seq); 
      #endif
      delete segment;
      segment = 0;
    
    } 
    
    /*
     * If we have messages in the queue, and there is no message in progress, 
     * get the message and start working on it. 
     */

    int size = cbuf.size();

        if (segment == 0){ // Last movement was completed and message deleted. 

      int size = cbuf.size();
      
      if( size > 0 ){ // There is a message waiting

        segment = cbuf.getSegment();

        #if(DEBUG_PRINT_ENABLED)
        if (segment != 0){
          Serial.print("Start#"); Serial.print(segment->seq); 
          Serial.print(" code="); Serial.print(segment->code); 
          Serial.print(" T="); Serial.print(segment->segment_time); 
          Serial.print(" ca="); Serial.print(segment->axes[0].ca); 
          Serial.print(" n="); Serial.print(segment->axes[0].n); 
          Serial.print(" x="); Serial.print(segment->axes[0].x); 
          Serial.println();
        }
        #endif

        if (segment->code == IDX_COMMAND_RESET){
 
          cbuf.sendResponseCode(*segment, IDX_RESPONSE_RESET);
          delete segment;
          segment = 0;
          continue;
        } else if (segment->code == IDX_COMMAND_LOAD){
          delete segment;
          segment = 0;
          continue;
        } else if (segment->code == IDX_COMMAND_RUN){
          delete segment;
          segment = 0;
          continue;
        } else{

          if(starvedToggle == true){
            starvedToggle=false;
            fastDebugClear(STARVED_TICK_PIN);
          }
          

          last_time = millis();

          
          
        }
      } else { // No message waiting, starved

          if(starvedToggle == false){
            starvedToggle=true;
            cbuf.sendEmpty();
            fastDebugSet(STARVED_TICK_PIN);   
          }
          continue;
      }
    } // end segment == 0



    
    cbuf.endLoop();

  }

  return 0;
}
