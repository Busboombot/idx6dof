/*
 *  Class for an IDX Robot Control Pendant interface
 */

#ifndef  idx_pendant_h
#define idx_pendant_h

#include "Arduino.h"
#include <stdint.h>

class IDXPendant {

  public:

  IDXPendant();

  void setup(); // Initialize the input and output pins
  bool run_once(); // Scan once and record which switches are set
  void print_serial(); // Write the two switch bit values to the serial port. 

  private:

    static const int outpins[];
    static const int inpins[];

    unsigned long swbits_ups; // bit mask of switches that are in the up position
    unsigned long swbits_downs; // bit mask of switches that are in the down position
    unsigned long last_swbits_ups; 
    unsigned long last_swbits_downs; 
    
    uint8_t switch_set_count; // number of switches set per loop 
    uint8_t last_switch_set_count;

    bool serial_is_setup;

  
};

#endif

