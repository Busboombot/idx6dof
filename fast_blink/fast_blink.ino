
#include "pins_arduino.h"


#define fastSet(pin) ( digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) ( digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) ) 

void setup() {
  pinMode(13, OUTPUT);
}

void loop() {

  fastSet(13);
  fastClear(13);
  fastSet(13);
  fastClear(13);
}
