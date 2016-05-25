/*
 *  Class for an IDX Robot Control Pendant interface
 */

#include "Arduino.h"
#include "idx_pendant.h"
#include "bithacks.h"

#include <stdio.h>
#include <stdint.h>

#define NUM_OUT_PINS 8
#define NUM_IN_PINS 6

#define NUM_SWITCHES 18 // 17+1 b/c array is 1-indexed



// TODO: These should probably be configurable, but since it's unlikely 
// I'll rebuild the hardware ... 
const int  IDXPendant::outpins[] = { 23, 25, 27, 29, 31, 33, 34, 35 };
const int  IDXPendant::inpins[] =  { 22, 24, 26, 28, 30, 32 };
const int  IDXPendant::num_switches = NUM_SWITCHES;

IDXPendant::IDXPendant() {
  
    this->serial_is_setup = false;
    
    this->swbits_ups = 0; // bit mask of switches that are in the up position
    this->swbits_downs = 0; // bit mask of switches that are in the down position
    
    this->last_swbits_ups = 0; 
    this->last_swbits_downs = 0; 

    this->last_switch_set_count = 0;
}


// the setup function runs once when you press reset or power the board

void IDXPendant::setup() {
  
  // initialize digital pin 13 as an output.
  for (int i = 0 ; i < NUM_OUT_PINS; i++){
    pinMode(this->outpins[i], OUTPUT);
  }

  for (int i = 0 ; i < NUM_IN_PINS; i++){
    pinMode(this->inpins[i], INPUT);
  }

}

#define SW_IS_UP(n) ( B_IS_SET(this->swbits_ups, n) )
#define SW_IS_DOWN(n)  ( B_IS_SET(this->swbits_downs, n) )

#define SET_SW_UP(n) ( B_SET(this->swbits_ups, n) )
#define SET_SW_DOWN(n)  ( B_SET(this->swbits_downs, n) )

// the loop function runs over and over again forever
bool IDXPendant::run_once() {

  switch_set_count = 0;
  
  this->swbits_ups = this->swbits_downs = 0;
  
  for (int i = 0 ; i < NUM_OUT_PINS; i++){
    digitalWrite(this->outpins[i], HIGH);  
    delay(1); // Delay needed to get out ping to stabilize
    for (int j = 0 ; j < NUM_IN_PINS; j++){
        if (digitalRead(this->inpins[j])){
          
          switch_set_count++;
          //Serial.print(this->outpins[i],DEC);
          //Serial.print("->");
          //Serial.println(this->inpins[j],DEC);
          
          switch (i+j*8) {
            case 4: SET_SW_UP(2); break; 
            case 12: SET_SW_DOWN(2); break; 
            case 16: SET_SW_UP(17); break; 
            case 17: SET_SW_UP(16); break; 
            case 18: SET_SW_UP(15); break; 
            case 19: SET_SW_UP(14); break; 
            case 20: SET_SW_UP(13); break; 
            case 21: SET_SW_UP(12); break; 
            case 22: SET_SW_UP(10); break; 
            case 23: SET_SW_UP(11); break; 
            case 24: SET_SW_DOWN(17); break; 
            case 25: SET_SW_DOWN(16); break; 
            case 26: SET_SW_DOWN(15); break; 
            case 27: SET_SW_DOWN(14); break; 
            case 28: SET_SW_DOWN(13); break; 
            case 29: SET_SW_DOWN(12); break; 
            case 30: SET_SW_DOWN(10); break; 
            case 31: SET_SW_DOWN(11); break; 
            case 32: SET_SW_UP(7); break; 
            case 33: SET_SW_UP(6); break; 
            case 34: SET_SW_UP(5); break; 
            case 35: SET_SW_UP(4); break; 
            case 36: SET_SW_UP(3); break; 
            case 37: SET_SW_UP(8); break; 
            case 38: SET_SW_UP(1); break; 
            case 39: SET_SW_UP(9); break; 
            case 40: SET_SW_DOWN(7); break; 
            case 41: SET_SW_DOWN(6); break; 
            case 42: SET_SW_DOWN(5); break; 
            case 43: SET_SW_DOWN(4); break; 
            case 44: SET_SW_DOWN(3); break; 
            case 45: SET_SW_DOWN(8); break; 
            case 46: SET_SW_DOWN(1); break; 
            case 47: SET_SW_DOWN(9); break; 
          }
        }
    }
    digitalWrite(this->outpins[i], LOW);  
    delay(1);  // Delay needed to get out ping to stabilize
  }

  int ret_val = (this->last_swbits_ups != swbits_ups || this->last_swbits_downs != swbits_downs);
 
  this->last_switch_set_count = switch_set_count;
  this->last_swbits_ups = this->swbits_ups;
  this->last_swbits_downs = this->swbits_downs;

  return ret_val;
  
}


void IDXPendant::print_serial() {

    if (! this->serial_is_setup){
        //Initialize serial and wait for port to open:
        Serial.begin(115200);
        while (!Serial) {
          ; // wait for serial port to connect. Needed for native USB port only
        }
        this->serial_is_setup = true;
    }
  
    Serial.print(this->last_switch_set_count, DEC);
    Serial.print(" ");

    Serial.print(this->swbits_ups, BIN);
    Serial.print(" ");
    Serial.print(this->swbits_downs, BIN);

    Serial.print('\n');
}

int IDXPendant::return_up() {
    return this->swbits_ups;
}

int IDXPendant::return_down() {
    return this->swbits_downs;
}


// Map the values fom the bit operation to 
static int sw_pos_map[] = { IDX_SW_POS_MID, IDX_SW_POS_TOP, IDX_SW_POS_BOTTOM, 0 };

int IDXPendant::sw_pos(int switch_n){

  return sw_pos_map[SW_IS_UP(switch_n) | (SW_IS_DOWN(switch_n)<<1)];
  
}




