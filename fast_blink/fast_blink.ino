
#include "pins_arduino.h"


#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) ) 

void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  #if defined (__arm__) && defined (__SAM3X8E__)
    fastSet(13);
    delay(250);
    fastClear(13);
    delay(250);
  #endif
}
