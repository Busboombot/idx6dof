#include <Arduino.h>
#include "Encoder.h"

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  int baud = 115200;
  Serial.begin(baud); 
  Serial1.begin(115200);
  delay(200);
  Serial1.printf("Starting %d baud\n", baud);


}

Encoder enc1(2,3);
uint8_t i = 0;
void loop() {

  Serial1.printf("%03d %d\n", i++, enc1.read());
  delay(500);
  
}
