/*
 *
 * Stepper code is: Copyright (C) 2009-2013 Mike McCauley
 * Use is subject to license conditions. The main licensing options available are GPL V2 or Commercial.
 * The stepper code uses speed calculations as described in 
 * "Generate stepper-motor speed profiles in real time" by David Austin 
 * http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf or
 * http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time or
 * http://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf

 * Parts of this code from:
 * Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 * See full license at end of file
 */
 
#include "digitalWriteFast.h"
#include "Arduino.h"
#include "direct_pin_read.h"
//#include "interrupt_config.h"


#define QUAD_PIN_A    2 // Has an interrupt on UNO
#define QUAD_PIN_B    3 // Has an interrupt on UNO


void setup() {

  attachInterrupt(digitalPinToInterrupt(QUAD_PIN_A), update_quadrature, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_PIN_B), update_quadrature, CHANGE);
  
  Serial.begin(115200);
  Serial.println("Starting");
  

  return;


}

void loop() {
  ;
}



void update_quadrature() {

  return;
 
}

