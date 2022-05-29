/*
6 Axis step generator and trajectory planner

Reads packets data from the serial port that represents 
6 axis motion segments, determines the optimal acelleration and
velocity for each segment, and generates step and direction pulses

*/
#include <Arduino.h>

#include <limits.h>


#include "trj_loop.h"
#include "trj_debug.h"

Stream &debug_serial = Serial1;

// These are supposed to be defined somewhere, but aren't
unsigned __exidx_start;
unsigned __exidx_end;

Loop mainLoop(Serial);

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  int baud = 115200;
  Serial.begin(baud); 
  Serial1.begin(115200);
  delay(200);
  Serial1.printf("Starting %d baud\n", baud);

  mainLoop.setup();
}

void loop(){
  mainLoop.loopOnce();
}
