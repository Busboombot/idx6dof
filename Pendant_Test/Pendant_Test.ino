#include "digitalWriteFast.h"
#include "idx_pendant.h"

IDXPendant pendant;

int s_vel[6];
int dst_s_vel[6];

uint8_t accel_clk = 0;

uint8_t stick_dir[6];
uint8_t stick_tus[6] = {5,5,5};
uint8_t stick_clk[6];
uint8_t stick_stp[6];

#define H_DEST_VEL 200
#define M_DEST_VEL 185
#define L_DEST_VEL 135

#define N_STICKS 6
#define ACCEL 2 // Inversly Porportional

uint8_t dir_pins[6] = {3,5,7};
uint8_t stp_pins[6] = {2,4,6};
#define STEP_PIN(stick) stp_pins[stick]
#define DIR_PIN(stick) dir_pins[stick]
#define MAX_TUS 1
#define S_SNAP_TOL 5

void setup() {
  pendant.setup();
  for (int stick = 0; stick < N_STICKS; stick ++) {
    pinMode(STEP_PIN(stick), OUTPUT);
    pinMode(DIR_PIN(stick), OUTPUT);
  }
  // Port Manipulations
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
  
  for (int stick = 0; stick < N_STICKS; stick ++) { // CLK Steps
    if (!(stick_clk[stick]--)) {
      stick_clk[stick] = stick_tus[stick];
      if (stick_tus[stick] != 0 && stick_tus[stick] < 50) {
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
      digitalWrite(STEP_PIN(stick), HIGH);
    }
    else if(stick_stp[stick] == 0) {
      digitalWrite(STEP_PIN(stick), LOW);
    }
    
    digitalWrite(DIR_PIN(stick), stick_dir[stick]);
    
    delayMicroseconds(12);
    
    for (int i = 0; i < N_STICKS; i ++) {
      digitalWrite(STEP_PIN(stick), LOW);
    }
    
    delayMicroseconds(12);
  }
}


void get_dest_vel() { // Reads button values and sets destination stepper velocity
  pendant.run_once();
  for (int stick = 0; stick < N_STICKS; stick ++) {
    
    if (pendant.sw_pos(stick+3) == 2) {
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
    else if (pendant.sw_pos(stick+3) == 0) {
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
}

void accel() {
  for (int stick = 0; stick < N_STICKS; stick ++) {
    if (s_vel[stick] < dst_s_vel[stick]) {
      s_vel[stick] += 1;
    }
    if (s_vel[stick] > dst_s_vel[stick]) {
      s_vel[stick] -= 1;
    }
    if (s_vel[stick] < (dst_s_vel[stick] + S_SNAP_TOL) && s_vel[stick] > (dst_s_vel[stick] - S_SNAP_TOL)) {
      s_vel[stick] = dst_s_vel[stick];
    }
  }
}

void dir_vel_set() {
  for (int stick = 0; stick < N_STICKS; stick ++) {
    if (s_vel[stick] > 0) {
      stick_dir[stick] = HIGH;
      stick_tus[stick] = map(s_vel[stick], 0, 200, 50, MAX_TUS);
    }
    else {
      stick_dir[stick] = LOW;
      stick_tus[stick] = map(s_vel[stick], 0, -200, 50, MAX_TUS);
    }
  }
}

void portWrite (int8_t port, int8_t state) {
  
}
