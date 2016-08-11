

#include "idx_pendant.h"


IDXPendant pendant  = IDXPendant();

void setup() {
  Serial.begin(115200);
  pendant.begin();
}


long lastTime = millis() - 500;

void loop() {
  if (pendant.run_once()){

    Serial.println(pendant.outstr());
  }

}
