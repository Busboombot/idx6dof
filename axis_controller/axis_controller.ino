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
#include "interrupt_pins.h"

#define QUAD_PIN_A    2 // Has an interrupt on UNO
#define QUAD_PIN_B    3 // Has an interrupt on UNO
#define INDEX_PIN     4
#define LIMIT_PIN     5
#define STEP_PIN      6
#define DIRECTION_PIN 7

typedef enum {
  DIRECTION_CCW = 0,  ///< Clockwise
  DIRECTION_CW  = 1   ///< Counter-Clockwise
} Direction;

typedef struct {
  volatile IO_REG_TYPE *pin1_register;
  volatile IO_REG_TYPE *pin2_register;
  IO_REG_TYPE pin1_bitmask;
  IO_REG_TYPE pin2_bitmask;
  volatile uint8_t state;
  volatile int8_t direction;
  volatile int32_t position; // Hmmmn, why aren't these volatile?
} Encoder_internal_state_t;

Encoder_internal_state_t encoder;

typedef struct {

  volatile IO_REG_TYPE *step_register;
  volatile IO_REG_TYPE *direction_register;
  IO_REG_TYPE step_bitmask;
  IO_REG_TYPE direction_bitmask;
  
  int8_t direction;
  int32_t position; // The current absolution position in steps. 


  long targetPos;  // Steps
  float speed;     // Steps per second
  float maxSpeed;  // Steps per second
  float acceleration;
  float sqrt_twoa; // Precomputed sqrt(2*_acceleration)
  unsigned long stepInterval; /// The current interval between steps in microseconds.
  unsigned long lastStepTime; /// The last step time in microseconds
  unsigned int   minPulseWidth;
  
  // Parameters from the equations in the David Austin paper.
  long n;     /// The step counter for speed calculations
  float c0;   /// Initial step size in microseconds
  float cn;   /// Last step size in microseconds
  float cmin; /// Min step size in microseconds based on maxSpeed

} stepper_t;

stepper_t stepper;

void setup() {

  attachInterrupt(digitalPinToInterrupt(QUAD_PIN_A), update_quadrature, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_PIN_B), update_quadrature, CHANGE);
  
  Serial.begin(115200);
  Serial.println("Starting");


  stepper.minPulseWidth = 1;

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  stepper.step_register = PIN_TO_BASEREG(STEP_PIN);
  stepper.step_bitmask = PIN_TO_BITMASK(STEP_PIN);
  stepper.direction_register = PIN_TO_BASEREG(DIRECTION_PIN);
  stepper.direction_bitmask = PIN_TO_BITMASK(DIRECTION_PIN);
  
  pinMode(INDEX_PIN, INPUT);
  pinMode(LIMIT_PIN, INPUT);
  pinMode(QUAD_PIN_A, INPUT_PULLUP);
  pinMode(QUAD_PIN_B, INPUT_PULLUP);

  encoder.position = 0;
  encoder.state = 0;
  encoder.direction = 0;
  
  encoder.pin1_register = PIN_TO_BASEREG(QUAD_PIN_A);
  encoder.pin1_bitmask = PIN_TO_BITMASK(QUAD_PIN_A);
  encoder.pin2_register = PIN_TO_BASEREG(QUAD_PIN_B);
  encoder.pin2_bitmask = PIN_TO_BITMASK(QUAD_PIN_B);

  read_initial_encoder_state();

  set_acceleration(1000);
  set_speed(5000);
  set_max_speed(5000);
  stepper.targetPos = 0;

  compute_new_speed();


  // "Pin Change" interrupts, using a library. 
  //enableInterrupt( INDEX_PIN, count_index, CHANGE);
  
}

int tick = 0;

void loop() {

  
  tick++;

  if (step()){
    
    delayMicroseconds( stepper.minPulseWidth);
    
    digitalWriteFast(STEP_PIN, LOW)
    
  }

  compute_new_speed();
  
  stepper.targetPos = encoder.position;

  if (tick % 1000 == 0){
    Serial.begin(115200);
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(stepper.position);
    Serial.print(" ");
    Serial.print(encoder.position);
    Serial.print("\n");
    Serial.end();
  }
 
}




void read_initial_encoder_state(){
    delayMicroseconds(2000);
    uint8_t s = 0;
    if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) {
      s |= 1;
    }
    if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) {
      s |= 2;
    }
    encoder.state = s;
}

void update_quadrature() {

  /// Code originally from: Paul Stoffregen <paul@pjrc.com>

  uint8_t p1val = DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask);
  uint8_t p2val = DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask);

  // Keep only the bottom two bits, the last values for A and B
  uint8_t state = encoder.state & 3;
  // Set the current tw values for A and B above the old ones.
  if (p1val){
    state |= 4;
  }
  if (p2val){
    state |= 8;
  }

  // For the next round, shift the old values off.
  encoder.state = (state >> 2);

  switch (state) {
    case 1:  // 1  = B0001
    case 7:  // 7  = B0111
    case 8:  // 8  = B1000
    case 14: // 14 = B1110
      encoder.position++;
      encoder.direction = 1;
      return;
    case 2:  // 2  = B0010
    case 4:  // 4  = B0100
    case 11: // 11 = B1011
    case 13: // 13 = B1101
      encoder.position--;
      encoder.direction = -1;
      return;
    // These are cases when only one pin has an interrupt
    case 3:  // 3  = B0011
    case 12: // 12 = B1100
      encoder.position += 2;
      encoder.direction = 1;
      return;
    case 6: // 6 = B0110
    case 9: // 9 = B1001
      encoder.position -= 2;
      encoder.direction = -1;
      return;
      // Unused, impossible states.
      // 0
      // 5  = B0101
      // 10 = B1010
      // 15 = B1111
  }
}

/* Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
