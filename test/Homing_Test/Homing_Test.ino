#include <Encoder.h>

Encoder encOne(2, 3);

#define STEP_PIN 6
#define DIR_PIN 7
#define LIMIT_PIN 5

#define QUAD_A 2
#define QUAD_B 3

#define BCK_LSH 100

uint16_t encCount;
uint16_t LimIndxDist;
volatile uint8_t indexTrig;

uint8_t quadDir;

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

  attachInterrupt(digitalPinToInterrupt(QUAD_A), QuadPulseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_B), QuadPulseB, CHANGE);
  
  for (uint8_t i=A0; i<=A5; i++) {
    pinMode(i, INPUT);
    digitalWrite(i,HIGH);
  }
  pciSetup(A0);

  Serial.begin(115200);
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
    switch (dir) {
      case 0: // Counterclockwise Result
        //=============================================================================
        switch (state) {
          case LOW:
            // Step CounterClockwise
            if (digitalRead(LIMIT_PIN) == HIGH) {
              // Stop Steps
              homeFound = 1;
            }
            break;
          case HIGH:
            // Step Clockwise
            if (digitalRead(LIMIT_PIN) == LOW) {
              for (int i = 0; i < BCK_LSH; i++) { // Continue stepping for a while to account for backlash
                // Step Clockwise
              }
              // Stop Steps
              state = 2; // Go back for backlash
            }
            break;
          case 2:
            // Step CounterClockwise
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
            if (digitalRead(LIMIT_PIN) == LOW) {
              // Stop Steps
              homeFound = 1;
            }
            break;
          case LOW:
            // Step CounterClockwise
            if (digitalRead(LIMIT_PIN) == HIGH) {
              for (int i = 0; i < BCK_LSH; i++) { // Continue stepping for a while to account for backlash
                // Step CounterClockwise
              }
              // Stop Steps
              state = 2; // Go back for backlash
            }
            break;
          case 2:
            // Step Clockwise
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
  encCount = 0; 
  
  while (distFound == 0) {
    switch (dir) {
      case 0:
        // Step CounterClockwise
        encCount = encOne.read();
        if (indexTrig != 0) {
          LimIndxDist = 
        }
        break;
      case 1:
        // Step Clockwise
        encCount = encOne.read();
        if (indexTrig != 0) {
          LimIndxDist = enc.read();
        }
        break;
    }
  }
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

uint8_t quadState;

void QuadPulseA() {
  if (quadState == 1 && digitalRead(QUAD_A) == HIGH) {
    quadDir = 0;
  }
  if (digitalRead(QUAD_A) == HIGH) {
    quadState = 0;
  }
}

void QuadPulseB() {
  if (quadState == 0 && digitalRead(QUAD_B) == HIGH) {
    quadDir = 1;
  }
  if (digitalRead(QUAD_V) == HIGH) {
    quadState = 1;
  }
}

