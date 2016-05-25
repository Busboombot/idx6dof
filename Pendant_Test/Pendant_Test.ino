/*

 */


#include "idx_pendant.h"

IDXPendant pendant;

int s_vel[6];
int dst_s_vel[6];

#define H_DEST_VEL 200
#define M_DEST_VEL 100
#define L_DEST_VEL 50

#define N_STICKS 6
#define ACCEL 3

void setup() {

  pendant.setup();
  Serial.begin(115200);
}


void loop() {  
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
