#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include "digitalWriteFast.h"

Adafruit_MCP23008 mcp;

int8_t dir;
int stick_val;
int aft_stick_val;
int8_t inverse_val;
int analog_val;

//uint8_t prev_dir[6] 


uint8_t dir_pins[6] = {3,5,7};
uint8_t stp_pins[6] = {2,4,6};
uint8_t analog_pins[6] = {0,1,2};

uint8_t stick_dir[6];
uint8_t stick_tus[6] = {5,5,5};
uint8_t stick_clk[6];

uint8_t stick_stp[6];

int analog_zero[6] = {0,0,0};

int8_t  limit_pins[6] =  {0,1,2}; // Limit switch inputs on the MCP23008
int8_t  limittrig_pins[6] =  {3,4,5}; // Limit switch trigger lights on the MCP23008
uint8_t limit_values[8];
uint8_t prev_dir[6];
uint8_t stop_state[6] = {0,0,0}

#define STEP_PIN(stick) stp_pins[stick]
#define DIR_PIN(stick) dir_pins[stick]
#define ANALOG_PIN(stick) analog_pins[stick]

#define N_STICKS 3
#define P_STICK_TOL 50
#define N_STICK_TOL -50

void setup() {
  //Serial.begin(9600);
  
  for (int stick = 0; stick < N_STICKS; stick ++) {
    pinModeFast(STEP_PIN(stick), OUTPUT);
    pinModeFast(DIR_PIN(stick), OUTPUT);
  }
  
  for ( int stick = 0; stick < N_STICKS ; stick ++ ){
    analog_zero[stick] = analogRead( ANALOG_PIN(stick)); 
  }
  mcp.begin();      // use default address 0

  for (int i = 0; i < N_STICKS; i++){
    mcp.pinMode(limit_pins[i], INPUT);
    mcp.pullUp(limit_pins[i], HIGH);  // turn on a 100K pullup internally
    
    mcp.pinMode(limittrig_pins[i], OUTPUT);
  }
}

void loop() {
  readAnalog();
  for (int stick = 0; stick < N_STICKS; stick ++) { // CLK Steps
    if (!(stick_clk[stick]--)) {
      stick_clk[stick] = stick_tus[stick];
      if (stick_tus[stick] != 0) {
        stick_stp[stick] = 1;
      }
      else {
        stick_stp[stick] = 0;
      }
    }
    else {
      stick_stp[stick] = 0;
    }
  }
  
  for (int stick = 0; stick < N_STICKS; stick ++) { // Write Steps and Directions
    if (stick_stp[stick] == 1) {
      digitalWriteFast2(STEP_PIN(stick), HIGH);
    }
    else if(stick_stp[stick] == 0) {
      digitalWriteFast2(STEP_PIN(stick), LOW);
    }
    
    digitalWriteFast2(DIR_PIN(stick), stick_dir[stick]);
    
    delayMicroseconds(12);
    
    for (int i = 0; i < N_STICKS; i ++) {
      digitalWriteFast2(STEP_PIN(stick), LOW);
    }
    
    delayMicroseconds(12);
  }
}

void readAnalog() {
  for (int stick = 0; stick < N_STICKS; stick ++) {
    limit_values[stick] = mcp.digitalRead(limit_pins[stick]);
    analog_val = analogRead(ANALOG_PIN(stick));
    stick_val = map(analog_val-analog_zero[stick], -700, 700, -500, 500);
    
    if (limit_values[stick] == HIGH && stop_state[stick] == 0) {
      if (stick_val > P_STICK_TOL) {
        prev_dir[stick] = HIGH;
        dir = HIGH;
        aft_stick_val = stick_val;
      }
      else if (stick_val < N_STICK_TOL) {
        prev_dir[stick] = LOW;
        dir = LOW;
        aft_stick_val = -stick_val;
      }
      else {
        // dir = LOW;
        aft_stick_val = 0;
      }
    }
    
    if (limit_values[stick] == LOW && stop_state[stick] == 0) {
      if (prev_dir[stick] == LOW) {
        dir = HIGH;
      }
      else {
        dir == LOW;
      }
      stop_state[stick] = 1;
    }
    
    
    if (stop_state[stick] == 1 && limit_values[stick] == HIGH) {
      aft_stick_val = 0;
      if (prev_dir[stick] == LOW && stick_val > P_STICK_TOL) {
        stop_state[stick] = 0;
      }
      else if (prev_dir[stick] == HIGH && stick_val < N_STICK_TOL) {
        stop_state[stick] = 0;
      }
    }
    
    stick_dir[stick] = dir;
    if (aft_stick_val == 0) {
      stick_tus[stick] = 0;
    }
    else {
      stick_tus[stick] = (700/aft_stick_val)*10;
    }
  }
}
