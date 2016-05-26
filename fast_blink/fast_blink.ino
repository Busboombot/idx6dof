
#include "pins_arduino.h"


#define fastWrite(pin) ( digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) ( digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) ) 

void setup() {
  pinMode(13, OUTPUT);
}

void loop() {

  fastWrite(13);
  fastClear(13);
  fastWrite(13);
  fastClear(13);
}
