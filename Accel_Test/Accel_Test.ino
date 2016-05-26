#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include "digitalWriteFast.h"

Adafruit_MCP23008 mcp;

int8_t dir;
int stick_val;
int aft_stick_val[6];
int8_t inverse_val;
int analog_val;

int int_vel[6];
int dst_vel[6];
int8_t stick_accel[6] = {1,1,1}; // Accelaration Levels

//uint8_t prev_dir[6]


uint8_t dir_pins[6] = {3,5,7};
uint8_t stp_pins[6] = {2,4,6};
uint8_t analog_pins[6] = {0,1,2};

uint8_t stick_dir[6];
uint8_t stick_tus[6] = {0,0,0};
uint8_t stick_clk[6];
uint8_t stick_stp[6];

int analog_zero[6] = {0,0,0};

int8_t  limit_pins[6] =  {0,1,2}; // Limit switch inputs on the MCP23008
int8_t  limittrig_pins[6] =  {3,4,5}; // Limit switch trigger lights on the MCP23008
uint8_t limit_values[8];

uint8_t sel_state;
uint8_t sel_bounce[6];

uint8_t d_velocity[6] = {0,0,0};

#define STEP_PIN(stick) stp_pins[stick]
#define DIR_PIN(stick) dir_pins[stick]
#define ANALOG_PIN(stick) analog_pins[stick]
#define SEL_PIN A3

#define N_STICKS 3
#define P_STICK_TOL 25
#define N_STICK_TOL -25

void setup() {
  // Serial.begin(115200);
  
  pinMode(SEL_PIN, INPUT_PULLUP);
  for (int stick = 0; stick < N_STICKS; stick ++) {
    pinModeFast(STEP_PIN(stick), OUTPUT);
    pinModeFast(DIR_PIN(stick), OUTPUT);
  }
  
  for ( int stick = 0; stick < N_STICKS ; stick ++ ){
    analog_zero[stick] = analogRead(ANALOG_PIN(stick)); 
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
      if (stick_tus[stick] != 0 && stick_tus[stick] != 62) {
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
    sel_state = digitalRead(SEL_PIN);
    analog_val = analogRead(ANALOG_PIN(stick));
    stick_val = map(analog_val-analog_zero[stick], -530, 530, 4000, -4000);
    
    // Serial.println(analog_val-analog_zero[stick]);
    
    if (sel_state == 0 && sel_bounce[stick] == 0) {
      if (stick_val > P_STICK_TOL) {
        dir = HIGH;
        dst_vel[stick] = stick_val;
        int_vel[stick] = aft_stick_val[stick];
      }
      else if (stick_val < N_STICK_TOL) {
        dir = LOW;
        dst_vel[stick] = -1*stick_val;
        int_vel[stick] = aft_stick_val[stick];
      }
      else {
        dir = LOW;
        dst_vel[stick] = 0;
        int_vel[stick] = aft_stick_val[stick];
      }
      sel_bounce[stick] = 1;
    }
    
    if (sel_state == 1) {
      sel_bounce[stick] = 0;  
    }
    //Serial.println(dst_vel[stick]);
    if (aft_stick_val[stick] != dst_vel[stick]) {
      if (dst_vel[stick] > int_vel[stick]) { // Going up
        if (aft_stick_val[stick] < dst_vel[stick]) {
          aft_stick_val[stick] += stick_accel[stick];
        }
        if (aft_stick_val[stick] >= dst_vel[stick]) {
          aft_stick_val[stick] = dst_vel[stick]; // When we hit desitination, it stays @ destination
        }
      }
      if (dst_vel[stick] < int_vel[stick]) { // Going Down
        if (aft_stick_val[stick] > dst_vel[stick]) {
          aft_stick_val[stick] -= stick_accel[stick];
        }
        if (aft_stick_val[stick] <= dst_vel[stick]) {
          aft_stick_val[stick] = dst_vel[stick];
        }
      }
//      Serial.println("---");
//      Serial.println(int_vel[stick]);
//      Serial.println(map(aft_stick_val[stick]>>6, 0, 4000>>6, 4000>>6, 0));
//      Serial.println(dst_vel[stick]);
    }
    stick_tus[stick] = map(aft_stick_val[stick]>>6, 0, 62, 62, 0);
    stick_dir[stick] = dir;
  }
}
