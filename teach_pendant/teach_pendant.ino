/*

 */


#include "idx_pendant.h"

IDXPendant pendant;

// the setup function runs once when you press reset or power the board
void setup() {

  pendant.setup();

}


// the loop function runs over and over again forever
void loop() {

  if(pendant.run_once()) {
    pendant.print_serial();
  }
  
}
