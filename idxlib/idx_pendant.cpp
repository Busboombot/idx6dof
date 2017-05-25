/*
 *  Class for an IDX Robot Control Pendant interface
 */

#include "Arduino.h"
#include "idx_pendant.h"
#include "bithacks.h"
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>

#define NUM_OUT_PINS 8
#define NUM_IN_PINS 6

class SwitchEntry {
  public:
    int sw_code = -1;
    int sw_num = -1;
    int out_pin = -1;
    int in_pin = -1; 
    int pos_code = -1;
  
    SwitchEntry(int sw_code, int sw_num, int out_pin, int in_pin, int pos_code):
      sw_code(sw_code), sw_num(sw_num), out_pin(out_pin), in_pin(in_pin), pos_code(pos_code)
    {
      
    }
  
} ;

SwitchEntry switches[] = {
  SwitchEntry(0,IDX_SW_AXIS4,2,14,IDX_SW_POS_BOTTOM),
  SwitchEntry(1,IDX_SW_AXIS3,3,14,IDX_SW_POS_BOTTOM),
  SwitchEntry(2,IDX_SW_AXIS2,4,14,IDX_SW_POS_BOTTOM),
  SwitchEntry(3,IDX_SW_AXIS1,5,14,IDX_SW_POS_BOTTOM),
  SwitchEntry(4,IDX_SW_AXIS0,6,14,IDX_SW_POS_BOTTOM),
  SwitchEntry(5,IDX_SW_AXIS5,7,14,IDX_SW_POS_BOTTOM),
  SwitchEntry(6,IDX_SW_TOOL_OPEN,8,14,IDX_SW_POS_BOTTOM),
  SwitchEntry(7,IDX_SW_SPEED,9,14,IDX_SW_POS_BOTTOM),
  SwitchEntry(8,IDX_SW_AXIS4,2,15,IDX_SW_POS_TOP),
  SwitchEntry(9,IDX_SW_AXIS3,3,15,IDX_SW_POS_TOP),
  SwitchEntry(10,IDX_SW_AXIS2,4,15,IDX_SW_POS_TOP),
  SwitchEntry(11,IDX_SW_AXIS1,5,15,IDX_SW_POS_TOP),
  SwitchEntry(12,IDX_SW_AXIS0,6,15,IDX_SW_POS_TOP),
  SwitchEntry(13,IDX_SW_AXIS5,7,15,IDX_SW_POS_TOP),
  SwitchEntry(14,IDX_SW_TOOL_OPEN,8,15,IDX_SW_POS_TOP),
  SwitchEntry(15,IDX_SW_SPEED,9,15,IDX_SW_POS_TOP),
  SwitchEntry(16,IDX_SW_AXIS,2,16,IDX_SW_POS_BOTTOM),
  SwitchEntry(17,IDX_SW_STEP,3,16,IDX_SW_POS_BOTTOM),
  SwitchEntry(18,IDX_SW_HOME,4,16,IDX_SW_POS_BOTTOM),
  SwitchEntry(19,IDX_SW_SET,5,16,IDX_SW_POS_BOTTOM),
  SwitchEntry(20,IDX_SW_WHERE,6,16,IDX_SW_POS_BOTTOM),
  SwitchEntry(21,IDX_SW_VAC_ON,7,16,IDX_SW_POS_BOTTOM),
  SwitchEntry(22,IDX_SW_TOOL_DETACH,8,16,IDX_SW_POS_BOTTOM),
  SwitchEntry(23,IDX_SW_TOOL_ON,9,16,IDX_SW_POS_BOTTOM),
  SwitchEntry(24,IDX_SW_AXIS,2,17,IDX_SW_POS_TOP),
  SwitchEntry(25,IDX_SW_STEP,3,17,IDX_SW_POS_TOP),
  SwitchEntry(26,IDX_SW_HOME,4,17,IDX_SW_POS_TOP),
  SwitchEntry(27,IDX_SW_SET,5,17,IDX_SW_POS_TOP),
  SwitchEntry(28,IDX_SW_WHERE,6,17,IDX_SW_POS_TOP),
  SwitchEntry(29,IDX_SW_VAC_ON,7,17,IDX_SW_POS_TOP),
  SwitchEntry(30,IDX_SW_TOOL_DETACH,8,17,IDX_SW_POS_TOP),
  SwitchEntry(31,IDX_SW_TOOL_ON,9,17,IDX_SW_POS_TOP),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(36,IDX_SW_TOOL,6,18,IDX_SW_POS_BOTTOM),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(-1,-1,-1,-1,-1),
  SwitchEntry(44,IDX_SW_TOOL,6,19,IDX_SW_POS_TOP),
};

// TODO: These should probably be configurable, but since it's unlikely 
// I'll rebuild the hardware ... 
const int  IDXPendant::outpins[] = { 2, 3, 4, 5, 6, 7, 8, 9 };
const int  IDXPendant::inpins[] =  { 14, 15, 16, 17, 18, 19 };
const int  IDXPendant::num_switches = IDX_NUM_SWITCHES;

const char IDXPendant::sw_names[] = "abcdefghijhlmnopq";
const char IDXPendant::sw_nulls[] = "12345678901234567";
  

IDXPendant::IDXPendant()
{

    last_switch_set_count = 0;
    
}


// the setup function runs once when you press reset or power the board

void IDXPendant::begin() {
  
  for (int i = 0 ; i < NUM_OUT_PINS; i++){
    pinMode(outpins[i], OUTPUT);
  }

  for (int i = 0 ; i < NUM_IN_PINS; i++){
    pinMode(inpins[i], INPUT);
  }
  
  for (int i = 0; i < IDX_NUM_SWITCHES; i++ ){
    switch_positions[i] = last_switch_positions[i] = IDX_SW_POS_MID;
  }
  

}

bool IDXPendant::run_once() {


  switch_set_count = 0;
  
  for (int i = 0; i < IDX_NUM_SWITCHES; i++ ){
    switch_positions[i] = IDX_SW_POS_MID;
  }
  
  for (int i = 0 ; i < NUM_OUT_PINS; i++){
    digitalWrite(outpins[i], HIGH);  
    delayMicroseconds(10); // Delay needed to get output pin to stabilize
    for (int j = 0 ; j < NUM_IN_PINS; j++){

        if (digitalRead(inpins[j])){
          
          switch_set_count++;
    
          SwitchEntry& se = switches[(outpins[i]-2) + (inpins[j]-14)*8];
          
          switch_positions[se.sw_num-1] = se.pos_code;
          
        }
    }
    digitalWrite(outpins[i], LOW);  
    delayMicroseconds(10); // Delay needed to get output pin to stabilize
  }
  
  last_switch_set_count = switch_set_count;

  bool changed = false;
  
  for (int i = 0; i < IDX_NUM_SWITCHES; i++ ){
    if (switch_positions[i] != last_switch_positions[i]){
      
      changed = true;
    }
    last_switch_positions[i] = switch_positions[i];
  }

  return changed;
  
}


// Map the values fom the bit operation to 
static int sw_pos_map[] = { IDX_SW_POS_MID, IDX_SW_POS_TOP, IDX_SW_POS_BOTTOM, 0 };

int IDXPendant::sw_pos(int switch_n){
  
  if (switch_n == 0){
    return IDX_SW_POS_INVALID; // Not a valid switch
  }
  
  return switch_positions[switch_n - 1];
  
}

char IDXPendant::sw_pos_name(int switch_n){
  
  int pos = sw_pos(switch_n);
  
  switch_n -- ;
  
  if (pos == IDX_SW_POS_TOP){
    return toupper(sw_names[switch_n]);
    
  } else if (pos == IDX_SW_POS_BOTTOM){
    return tolower(sw_names[switch_n]);
    
  } else {
    return sw_nulls[switch_n];
    
  }
 
}

char outs[IDX_NUM_SWITCHES+1] = {0};

char* IDXPendant::outstr(){

  for (int i = 1; i <= IDX_NUM_SWITCHES; i++){
    outs[i-1] = sw_pos_name(i);
  }
  
  outs[IDX_NUM_SWITCHES] = 0;
  
  return outs;
  
}


