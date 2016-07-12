
#include "idx_pendant.h"

#define NUM_BUTTONS 17

String dataString = "";

IDXPendant pendant;

void setup() {
  
  pendant.begin();
  Serial.begin(9600);
  
}

void loop() {
  dataString = "";
  if(pendant.run_once()) {
    for (int i = 1; i <= NUM_BUTTONS; i++) {
      if (pendant.sw_pos(i) == IDX_SW_POS_TOP) {
        dataString += "2";
      }
      else if (pendant.sw_pos(i) == IDX_SW_POS_MID) {
        dataString += "1";
      }
      else if (pendant.sw_pos(i) == IDX_SW_POS_BOTTOM) {
        dataString += "0";
      }
    }
  }
  Serial.println(dataString);
}
