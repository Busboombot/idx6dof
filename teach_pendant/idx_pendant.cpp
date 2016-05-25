/*
 *  Class for an IDX Robot Control Pendant interface
 */

#include "Arduino.h"
#include "idx_pendant.h"
#include <stdio.h>
#include <stdint.h>

#define NUM_OUT_PINS 8
#define NUM_IN_PINS 6

#define NUM_SWITCHES 18 // 17+1 b/c array is 1-indexed

// TODO: These should probably be configurable, but since it's unlikely 
// I'll rebuild the hardware ... 
const int  IDXPendant::outpins[] = { 23, 25, 27, 29, 31, 33, 34, 35 };
const int  IDXPendant::inpins[] =  { 22, 24, 26, 28, 30, 32 };

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

// the loop function runs over and over again forever
bool IDXPendant::run_once() {

  switch_set_count = 0;
  
  this->swbits_ups = this->swbits_downs = 0;
  
  for (int i = 0 ; i < NUM_OUT_PINS; i++){
    digitalWrite(this->outpins[i], HIGH);  
    for (int j = 0 ; j < NUM_IN_PINS; j++){
        if (digitalRead(this->inpins[j])){
          delay(1);
          switch_set_count++;

          switch (i+j*8) {
            case 4:  this->swbits_ups |= ( 1 << 2 ); break;
            case 12: this->swbits_downs |= ( 1 << 2 ); break;
            case 16: this->swbits_ups |= ( 1 << 17 ); break;
            case 17: this->swbits_ups |= ( 1 << 16 ); break;
            case 18: this->swbits_ups |= ( 1 << 15 ); break;
            case 19: this->swbits_ups |= ( 1 << 14 ); break;
            case 20: this->swbits_ups |= ( 1 << 13 ); break;
            case 21: this->swbits_ups |= ( 1 << 12 ); break;
            case 22: this->swbits_ups |= ( 1 << 10 ); break;
            case 23: this->swbits_ups |= ( 1 << 11 ); break;
            case 24: this->swbits_downs |= ( 1 << 17 ); break;
            case 25: this->swbits_downs |= ( 1 << 16 ); break;
            case 26: this->swbits_downs |= ( 1 << 15 ); break;
            case 27: this->swbits_downs |= ( 1 << 14 ); break;
            case 28: this->swbits_downs |= ( 1 << 13 ); break;
            case 29: this->swbits_downs |= ( 1 << 12 ); break;
            case 30: this->swbits_downs |= ( 1 << 10 ); break;
            case 31: this->swbits_downs |= ( 1 << 11 ); break;
            case 32: this->swbits_ups |= ( 1 << 7 ); break;
            case 33: this->swbits_ups |= ( 1 << 6 ); break;
            case 34: this->swbits_ups |= ( 1 << 5 ); break;
            case 35: this->swbits_ups |= ( 1 << 4 ); break;
            case 36: this->swbits_ups |= ( 1 << 3 ); break;
            case 37: this->swbits_ups |= ( 1 << 8 ); break;
            case 38: this->swbits_ups |= ( 1 << 1 ); break;
            case 39: this->swbits_ups |= ( 1 << 9 ); break;
            case 40: this->swbits_downs |= ( 1 << 7 ); break;
            case 41: this->swbits_downs |= ( 1 << 6 ); break;
            case 42: this->swbits_downs |= ( 1 << 5 ); break;
            case 43: this->swbits_downs |= ( 1 << 4 ); break;
            case 44: this->swbits_downs |= ( 1 << 3 ); break;
            case 45: this->swbits_downs |= ( 1 << 8 ); break;
            case 46: this->swbits_downs |= ( 1 << 1 ); break;
            case 47: this->swbits_downs |= ( 1 << 9 ); break;
          }
        }
    }
    digitalWrite(this->outpins[i], LOW);    
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

    Serial.print(this->swbits_ups, DEC);
    Serial.print(" ");
    Serial.print(this->swbits_downs, DEC);

    Serial.print('\n');
}




