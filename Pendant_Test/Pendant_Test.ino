/*

 */


#include "idx_pendant.h"

IDXPendant pendant;

int s_vel[6];
int dst_s_vel[6];

uint8_t accel_clk = 0;

#define H_DEST_VEL 200
#define M_DEST_VEL 100
#define L_DEST_VEL 50

#define N_STICKS 6
#define ACCEL 1 // Inversly Porportional

#define S_SNAP_TOL 5

void setup() {

  pendant.setup();
  Serial.begin(115200);
}


void loop() {
  get_dest_vel();
  
  /*
  if(pendant.run_once()) {
    Serial.print("Bits Up: ");
    Serial.print(pendant.return_up(), BIN);
    Serial.print("--------Bits Down: ");
    Serial.print(pendant.return_down(), BIN);
    Serial.println("");
  }
  pendant.run_once();
  for (int button = 1; button < pendant.num_switches; button ++) {
    Serial.print(pendant.sw_pos(button));
  }
  Serial.println("");
  */
  if (accel_clk == 0) {
    accel();
  }
  
  accel_clk += 1;
  
  if (accel_clk >= ACCEL) {
    accel_clk = 0;
  }
  
  print_vel();
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

void print_vel() {
  for (int stick = 0; stick < N_STICKS; stick ++) {
    Serial.println(s_vel[stick]);
  }
  Serial.println("----------");
}
