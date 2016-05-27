#include "sam.h"


uint32_t BBITS_B;
uint32_t BBITS_C;
// Digital 0~7 set to outputs, then on/off using port manipulation

void setup() {
  REG_PIOB_OER = (1u<<27);
}
 
void loop() {
  REG_PIOB_SODR = (1u<<27);
  delay(500);
  REG_PIOB_CODR = (1u<<27);
  delay(500);
}
