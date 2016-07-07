/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

uint16_t encCount;

uint8_t indexTrig;

uint16_t clk;

Encoder encOne(2, 3);

#define DELAY 10000

#define TOL 100 // Tolerence for encoder snap

void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT1_vect) {
     indexTrig = 1;
}


void setup() {
  for (i=A0; i<=A5; i++) {
    pinMode(i, INPUT);
    digitalWrite(i,HIGH);
  }
  
  Serial.begin(115200);
   
}

void loop() {

  encCount = encOne.read(); // W/o PWM-202kHz | W/ PWM-202kHz | W/Inter-727kHz

  if (indexTrig == 1) {
    if (encCount % 8192 
  }
  
  if (clk >= DELAY) {
    Serial.println(encCount);
  }
  clk++;
}
