#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include "digitalWriteFast.h"
#include "idx_pendant.h"

IDXPendant pendant;

Adafruit_MCP23008 mcp;
uint8_t limit_pins[6] =  {2,1,0}; // Limit switch inputs on the MCP23008
uint8_t limit_values[8];
uint8_t limit_state[8];
uint8_t step_b_state;

int cur_s_vel[6];
int dst_s_vel[6];

uint8_t accel_clk = 0;

uint8_t stick_dir[6];
uint8_t stick_tus[6];
uint8_t stick_clk[6];
uint8_t stick_stp[6];
uint8_t prev_s_dir[6];
uint8_t add_stps_cnt[6];
#define EXTRA_LIMIT_STPS 3

uint8_t axis[6] = {IDX_SW_AXIS0, IDX_SW_AXIS1, IDX_SW_AXIS2, IDX_SW_AXIS3, IDX_SW_AXIS4, IDX_SW_AXIS5};

#define H_DEST_VEL 200
#define M_DEST_VEL 185
#define L_DEST_VEL 135

#define N_AXIS 6
#define ACCEL 1 // Inversly Porportional
#define ACCEL_STEP 3 // Porportional, ust be less than S_SNAP_TOL
#define HIGH_TUS_SUB 150 // Increases accelaration in this range from 0
#define HIGH_TUS_SUB_MULT 4 // Acceration Multiplyer (<<) in sub zone

uint8_t dir_pins[6] = {3,5,7};
uint8_t stp_pins[6] = {2,4,6};
#define STEP_PIN(stick) stp_pins[stick]
#define DIR_PIN(stick) dir_pins[stick]
#define MAX_TUS 1
#define MIN_TUS 30
#define S_SNAP_TOL 20
#define LIM_OVER_VEL 100 // Override velocity for limits

#if defined (__SAM3X8E__)
  // Defines Fastwrite functions for Arduino Due
  #define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
  #define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )
#else
  // Defines Fastwrite functions for other Arduinos
  #define fastSet(pin) (digitalFastWrite2(pin, HIGH))
  #define fastClear(pin) (digitalFastWrite2(pin, LOW))
#endif

void setup() {
  Serial.begin(115200);
  pendant.setup();
  mcp.begin();
  
  for (int stick = 0; stick < N_AXIS; stick ++) {
    pinMode(STEP_PIN(stick), OUTPUT);
    pinMode(DIR_PIN(stick), OUTPUT);
  }
  
  for (int i = 0; i < N_AXIS; i++){
    mcp.pinMode(limit_pins[i], INPUT);
    mcp.pullUp(limit_pins[i], HIGH);  // turn on a 100K pullup internally
  }
}

void loop() {
  get_dest_vel();
  if (accel_clk == 0) {
    accel();
  }
  
  accel_clk += 1;
  
  if (accel_clk >= ACCEL) {
    accel_clk = 0;
  }
  
  dir_vel_set();
  
  for (int stick = 0; stick < N_AXIS; stick ++) { // CLK Steps
    if (!(stick_clk[stick]--)) {
      stick_clk[stick] = stick_tus[stick];
      if (stick_tus[stick] != 0 && stick_tus[stick] < MIN_TUS) {
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
  for (int stick = 0; stick < N_AXIS; stick ++) { // Write Steps and Directions
    if (stick_stp[stick] == 1) {
      fastSet(STEP_PIN(stick));
      add_stps_cnt[stick] += 1;
    }
    else if(stick_stp[stick] == 0) {
      fastClear(STEP_PIN(stick));
    }
    
    
    if (stick_dir[stick] == HIGH) {
      fastSet(DIR_PIN(stick));
    }
    else {
      fastClear(DIR_PIN(stick));
    }
    
    delayMicroseconds(4);
    
    for (int i = 0; i < N_AXIS; i ++) {
      fastClear(STEP_PIN(stick));
    }
    
    delayMicroseconds(4);
  }
}

void get_dest_vel() { // Reads button values and sets destination stepper velocity
  pendant.run_once();
  
  step_b_state = pendant.sw_pos(IDX_SW_STEP); // Read Step Button State
  
  for (int stick = 0; stick < N_AXIS; stick ++) {
    limit_values[stick] = mcp.digitalRead(limit_pins[stick]);
    
    if (limit_state[stick] == 0) {
      if (pendant.sw_pos(axis[stick]) == 2) {
        if (pendant.sw_pos(IDX_SW_SPEED) == 0) {
          dst_s_vel[stick] = L_DEST_VEL;
        }
        else if (pendant.sw_pos(IDX_SW_SPEED) == 1) {
          dst_s_vel[stick] = M_DEST_VEL;
        }
        else {
          dst_s_vel[stick] = H_DEST_VEL;
        }
      }
      else if (pendant.sw_pos(axis[stick]) == 0) {
        if (pendant.sw_pos(IDX_SW_SPEED) == 0) {
          dst_s_vel[stick] = -L_DEST_VEL;
        }
        else if (pendant.sw_pos(IDX_SW_SPEED) == 1) {
          dst_s_vel[stick] = -M_DEST_VEL;
        }
        else {
          dst_s_vel[stick] = -H_DEST_VEL;
        }
      }
      else {
        dst_s_vel[stick] = 0;
      }
    }
    
    //Interupt
    if (limit_values[stick] == 0 && step_b_state != 0 && limit_state[stick] == 0) {
      if (cur_s_vel[stick] > 0) {
        prev_s_dir[stick] = HIGH;
      }
      else {
        prev_s_dir[stick] = LOW;
      }
      limit_state[stick] = 1;
    }
    if (cur_s_vel[stick] == dst_s_vel[stick] && limit_state[stick] == 1) {
      if (prev_s_dir[stick] == HIGH) {
        dst_s_vel[stick] = LIM_OVER_VEL * -1;
      }
      else {
        dst_s_vel[stick] = LIM_OVER_VEL;
      }
      limit_state[stick] = 2;
    }
    if (limit_state[stick] == 2 && limit_values[stick] == 1) {
      limit_state[stick] = 3;
    }
    if (limit_state[stick] == 3 && add_stps_cnt[stick] >= EXTRA_LIMIT_STPS) {
      limit_state[stick] = 0;
      add_stps_cnt[stick] = 0;
    }
    Serial.println(limit_state[stick]);
  }
}

void accel() {
  for (int stick = 0; stick < N_AXIS; stick ++) {
    if (cur_s_vel[stick] < dst_s_vel[stick]) {
      if (cur_s_vel[stick] < HIGH_TUS_SUB && cur_s_vel[stick] > -HIGH_TUS_SUB) {
        cur_s_vel[stick] += ACCEL_STEP<<HIGH_TUS_SUB_MULT;
      }
      else {
        cur_s_vel[stick] += ACCEL_STEP;
      }
    }
    if (cur_s_vel[stick] > dst_s_vel[stick]) {
      if (cur_s_vel[stick] < HIGH_TUS_SUB && cur_s_vel[stick] > -HIGH_TUS_SUB) {
        cur_s_vel[stick] -= ACCEL_STEP<<HIGH_TUS_SUB_MULT;
      }
      else {
        cur_s_vel[stick] -= ACCEL_STEP;
      }
    }
    if (cur_s_vel[stick] < (dst_s_vel[stick] + S_SNAP_TOL) && cur_s_vel[stick] > (dst_s_vel[stick] - S_SNAP_TOL)) {
      cur_s_vel[stick] = dst_s_vel[stick];
    }
  }
}

void dir_vel_set() {
  for (int stick = 0; stick < N_AXIS; stick ++) {
    if (cur_s_vel[stick] > 0) {
      stick_dir[stick] = HIGH;
      stick_tus[stick] = map(cur_s_vel[stick], 0, 200, MIN_TUS, MAX_TUS);
    }
    else {
      stick_dir[stick] = LOW;
      stick_tus[stick] = map(cur_s_vel[stick], 0, -200, MIN_TUS, MAX_TUS);
    }
  }
}
