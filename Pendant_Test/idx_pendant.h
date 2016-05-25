/*
 *  Class for an IDX Robot Control Pendant interface
 */

#ifndef  idx_pendant_h
#define idx_pendant_h

#include "Arduino.h"
#include <stdint.h>

#define IDX_SW_SPEED 1
#define IDX_SW_TOOL 2
#define IDX_SW_AXIS0 3
#define IDX_SW_AXIS1 4
#define IDX_SW_AXIS2 5
#define IDX_SW_AXIS3 6 
#define IDX_SW_AXIS4 7
#define IDX_SW_AXIS5 8 
#define IDX_SW_TOOL_OPEN 9 
#define IDX_SW_TOOL_ON 10 
#define IDX_SW_TOOL_DETACH 11
#define IDX_SW_VAC_ON 12
#define IDX_SW_WHERE 13
#define IDX_SW_SET 14
#define IDX_SW_HOME 15
#define IDX_SW_STEP 16
#define IDX_SW_AXIS 17

#define IDX_SW_POS_TOP 2
#define IDX_SW_POS_MID 1
#define IDX_SW_POS_BOTTOM 0

class IDXPendant {

  public:

  IDXPendant();

  static const int num_switches;

  void setup(); // Initialize the input and output pins
  
  bool run_once(); // Scan once and record which switches are set
  
  void print_serial(); // Write the two switch bit values to the serial port.
  
  int return_up(); // Return the binary values for all switches in the up position
  
  int return_down(); // Return the binary values for all switches in the down position
  
  int sw_pos(int switch_n); // Return the position of a switch

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

