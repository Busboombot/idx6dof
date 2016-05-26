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
#define ACCEL 1 // Inversly Porportional
#define ACCEL_STEP 3 // Porportional, ust be less than S_SNAP_TOL
#define HIGH_TUS_SUB 100 // Increases accelaration in this range from 0
#define HIGH_TUS_SUB_MULT 2 // Acceration Multiplyer (<<) in sub zone

uint8_t dir_pins[6] = {3,5,7};
uint8_t stp_pins[6] = {2,4,6};
#define STEP_PIN(stick) stp_pins[stick]
#define DIR_PIN(stick) dir_pins[stick]
#define MAX_TUS 1
#define MIN_TUS 30
#define S_SNAP_TOL 5

#if defined (__arm__) && defined (__SAM3X8E__)
  // Defines Fastwrite functions for Arduino Due
  #define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
  #define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) ) 
#endif

void setup() {
//  Serial.begin(115200);
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
  for (int stick = 0; stick < N_STICKS; stick ++) { // Write Steps and Directions
    if (stick_stp[stick] == 1) {
      #if defined(__arm__) && defined (__SAM3X8E__)
        fastSet(STEP_PIN(stick));
      #else
        digitalFastWrite2(STEP_PIN(stick), HIGH);
      #endif
    }
    else if(stick_stp[stick] == 0) {
      #if defined(__arm__) && defined (__SAM3X8E__)
        fastClear(STEP_PIN(stick));
      #else
        digitalFastWrite2(STEP_PIN(stick), LOW);
      #endif
    }
    
    #if defined(__arm__) && defined (__SAM3X8E__)
      if (stick_dir[stick] == HIGH) {
        fastSet(DIR_PIN(stick));
      }
      else {
        fastClear(DIR_PIN(stick));
      }
    #else
      digitalFastWrite2(DIR_PIN(stick), stick_dir[stick]);
    #endif
    delayMicroseconds(12);
    
    for (int i = 0; i < N_STICKS; i ++) {
      #if defined(__arm__) && defined (__SAM3X8E__)
        fastClear(STEP_PIN(stick));
      #else
        digitalFastWrite2(STEP_PIN(stick), LOW);
      #endif
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
      if (s_vel[stick] < HIGH_TUS_SUB && s_vel[stick] > -HIGH_TUS_SUB) {
        s_vel[stick] += ACCEL_STEP<<HIGH_TUS_SUB_MULT;
      }
      else {
        s_vel[stick] += ACCEL_STEP;
      }
    }
    if (s_vel[stick] > dst_s_vel[stick]) {
      if (s_vel[stick] < HIGH_TUS_SUB && s_vel[stick] > -HIGH_TUS_SUB) {
        s_vel[stick] -= ACCEL_STEP<<HIGH_TUS_SUB_MULT;
      }
      else {
        s_vel[stick] -= ACCEL_STEP;
      }
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
      stick_tus[stick] = map(s_vel[stick], 0, 200, MIN_TUS, MAX_TUS);
    }
    else {
      stick_dir[stick] = LOW;
      stick_tus[stick] = map(s_vel[stick], 0, -200, MIN_TUS, MAX_TUS);
    }
//    Serial.println(stick_tus[stick]);
  }
//  Serial.println("------");
}
