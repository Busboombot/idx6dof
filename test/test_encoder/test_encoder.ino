/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

int encCount;

uint16_t clk;

Encoder encOne(4, 5);

#define DELAY 1000

void setup() {
  Serial.begin(115200);
   
}

void loop() {

  encCount = encOne.read(); // W/o PWM-202kHz | W/ PWM-202kHz | W/Inter-727kHz
  
  if (clk >= DELAY) {
    Serial.println(encCount);
  }
  clk++;
}
