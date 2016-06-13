/*

 */


#include "idx_pendant.h"

IDXPendant pendant;

void setup() {

  pendant.setup();

}


void loop() {

  if(pendant.run_once()) {
    pendant.print_serial();

    if (pendant.sw_pos(IDX_SW_SPEED) == IDX_SW_POS_TOP){
      Serial.println("Fast");
    } else if (pendant.sw_pos(IDX_SW_SPEED) == IDX_SW_POS_MID){
      Serial.println("Medium");
    } else if (pendant.sw_pos(IDX_SW_SPEED) == IDX_SW_POS_BOTTOM){
      Serial.println("Slow");
    }
  
    if (pendant.sw_pos(IDX_SW_AXIS0) == IDX_SW_POS_TOP){
      Serial.println("Increase");
    } else if (pendant.sw_pos(IDX_SW_AXIS0) == IDX_SW_POS_MID){
      Serial.println("No Change");
    } else if (pendant.sw_pos(IDX_SW_AXIS0) == IDX_SW_POS_BOTTOM){
      Serial.println("Decrease");
    }
  }
  
}
