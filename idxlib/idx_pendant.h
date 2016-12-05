/*
 *  Class for an IDX Robot Control Pendant interface
 */

#ifndef  idx_pendant_h
#define idx_pendant_h

#include "Arduino.h"
#include <stdint.h>

#define IDX_NUM_SWITCHES 17

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
#define IDX_SW_POS_INVALID -1

class IDXPendant {

  public:

      IDXPendant();

      static const int num_switches;

      void begin(); // Initialize the input and output pins
  
      bool run_once(); // Scan once and record which switches are set
  
      int sw_pos(int switch_n); // Return the position of a switch

      char sw_pos_name(int switch_n); // Return the position of a switch

      char* outstr(); // Return the encoded output string

  private:

    static const int outpins[];
    static const int inpins[];

    static const char sw_names[];
    static const char sw_nulls[];

    uint8_t switch_positions[IDX_NUM_SWITCHES] = {IDX_SW_POS_MID};
    uint8_t last_switch_positions[IDX_NUM_SWITCHES] = {IDX_SW_POS_MID};
    
    uint8_t switch_set_count; // number of switches set per loop 
    uint8_t last_switch_set_count;

    bool serial_is_setup;

  
};

#endif

