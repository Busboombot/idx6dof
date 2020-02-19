/*
 * Use either the digitalFastWrite2 macros, or port manipulation that is specific to the Due
 */
 

#include <Arduino.h>
 
#ifndef trj_fastset_h
#define trj_fastset_h
 
#if defined (__SAM3X8E__)
  // Defines Fastwrite functions for Arduino Due
  #define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
  #define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )
#elif defined(TESTING)
#define digitalWriteFast(x,y)
#define fastSet(x)
#define fastClear(x)
#else
// Maybe Teensy
  #define fastSet(pin) digitalWriteFast(pin, LOW) 
  #define fastClear(pin) digitalWriteFast(pin, HIGH) 
  
#endif

#endif // idx_fastset_h

