#include "digitalWriteFast.h"
#include "Arduino.h"
#include "direct_pin_read.h"
#include "interrupt_pins.h"

#define STEP_PIN 6
#define DIR_PIN 7
#define LIMIT_PIN 5

#define QUAD_A 2
#define QUAD_B 3

#define BCK_LSH 100

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

uint16_t LimIndxDist;
volatile uint8_t indexTrig;

int8_t vel;
uint8_t stepClk;

void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT1_vect) {
  indexTrig++;
}

void setup() {

  attachInterrupt(digitalPinToInterrupt(QUAD_A), update_quadrature, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_B), update_quadrature, CHANGE);
  
  for (uint8_t i=A0; i<=A5; i++) {
    pinMode(i, INPUT);
    digitalWrite(i,HIGH);
  }
  pciSetup(A0);

  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  encoder.pin1_register = PIN_TO_BASEREG(QUAD_A);
  encoder.pin1_bitmask = PIN_TO_BITMASK(QUAD_A);
  encoder.pin2_register = PIN_TO_BASEREG(QUAD_B);
  encoder.pin2_bitmask = PIN_TO_BITMASK(QUAD_B);

  encoder.position = 0;
  encoder.state = 0;
  encoder.direction = 0;
  
  vel = 5; // Set Velocity to 5 cycles per step
  
  findHome(1);
}

void loop() {
  
}

void findHome(uint8_t dir) { // Function Tailored for Elbow 1 Axis
  uint8_t intLimVal = digitalRead(LIMIT_PIN);
  uint8_t homeFound = 0;
  uint8_t distFound = 0;
  uint8_t state = intLimVal;
  
  while (homeFound == 0) { // Loop until home is found
    Serial.println("Homing");
    switch (dir) {
      case 0: // Counterclockwise Result
        //=============================================================================
        switch (state) {
          case LOW:
            // Step CounterClockwise
            vel = abs(vel)*-1;
            step();
            if (digitalRead(LIMIT_PIN) == HIGH) {
              // Stop Steps
              homeFound = 1;
            }
            break;
          case HIGH:
            // Step Clockwise
            vel = abs(vel);
            step();
            if (digitalRead(LIMIT_PIN) == LOW) {
              for (int i = 0; i < BCK_LSH; i++) { // Continue stepping for a while to account for backlash
                // Step Clockwise
                vel = abs(vel);
                step();
              }
              // Stop Steps
              state = 2; // Go back for backlash
            }
            break;
          case 2:
            // Step CounterClockwise
            vel = abs(vel)*-1;
            step();
            if (digitalRead(LIMIT_PIN) == HIGH) {
              // Stop Steps
              homeFound = 1;
            }
            break;
        }
        break;
      case 1: // Clockwise Result
        //=============================================================================
        switch (state) {
          case HIGH:
            // Step Clockwise
            vel = abs(vel);
            step();
            if (digitalRead(LIMIT_PIN) == LOW) {
              // Stop Steps
              homeFound = 1;
            }
            break;
          case LOW:
            // Step CounterClockwise
            vel = abs(vel)*-1;
            step();
            if (digitalRead(LIMIT_PIN) == HIGH) {
              for (int i = 0; i < BCK_LSH; i++) { // Continue stepping for a while to account for backlash
                // Step CounterClockwise
                vel = abs(vel)*-1;
                step();
              }
              // Stop Steps
              state = 2; // Go back for backlash
            }
            break;
          case 2:
            // Step Clockwise
            vel = abs(vel);
            step();
            if (digitalRead(LIMIT_PIN) == LOW) {
              // Stop Steps
              homeFound = 1;
            }
            break;
        }
        break;
    }
  }
  
  indexTrig = 0;
  encoder.position = 0; 
  
  while (distFound == 0) {
    switch (dir) {
      case 0:
        // Step CounterClockwise
        vel = abs(vel)*-1;
        step();
        if (indexTrig != 0) {
          LimIndxDist = abs(encoder.position);
          distFound = 1;
        }
        break;
      case 1:
        // Step Clockwise
        vel = abs(vel);
        step();
        if (indexTrig != 0) {
          LimIndxDist = abs(encoder.position);
          distFound = 1;
        }
        break;
    }
  }
  indexTrig = 0;
  Serial.println(LimIndxDist);
  Serial.println("Homing Complete");
}

void step() {
  if (stepClk >= abs(vel)) {
    stepClk = 0;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(4);
    digitalWrite(STEP_PIN, LOW);
  }
  stepClk++;
  if (vel > 0) {
    digitalWrite(DIR_PIN, HIGH);
  }
  else if (vel < 0) {
    digitalWrite(DIR_PIN, LOW);
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
      encoder.position ++;
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

