/*
 * Use either the digitalFastWrite2 macros, or port manipulation that is specific to the Due
 */
 

#include "Arduino.h"
 
#if defined (__SAM3X8E__)
  // Defines Fastwrite functions for Arduino Due
  #define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
  #define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )
#else
  #include "digitalWriteFast.h"
  // Defines Fastwrite functions for other Arduinos

  #define fastSet(pin) (digitalFastWrite2(pin, HIGH))
  #define fastClear(pin) (digitalFastWrite2(pin, LOW))
#endif