
uint8_t indexTrig;
#define LIMIT_PIN 5
#define BCK_LSH 100

void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT1_vect) {
  indexTrig++;
}

void setup() {
  for (uint8_t i=A0; i<=A5; i++) {
    pinMode(i, INPUT);
    digitalWrite(i,HIGH);
  }
  pciSetup(A0);
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
// Set Encoder Value to 0
  
  while (distFound == 0) {
    switch (dir) {
      case 0:
        // Step CounterClockwise
        // Read Encoder Value
        if (indexTrig != 0) {
          // Set d to Encoder Value
        }
        break;
      case 1:
        // Step Clockwise
        // Read Encoder Value
        if (indexTrig != 0) {
          // Set d to Encoder Value
        }
        break;
    }
  }
}

