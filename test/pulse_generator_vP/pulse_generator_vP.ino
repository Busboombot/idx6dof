#include <digitalWriteFast.h>

#define N_PULSES 50
int8_t pins[] = {3,4,5,6,7,8,9,10,11};

void setup() {

  for(int i =0; i < sizeof pins; i++){
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

int32_t tick = N_PULSES;
void loop() {

  tick--;  
  for(int8_t i = 0; i < sizeof pins; i ++){
    digitalWrite(pins[i], HIGH);
  }
  delayMicroseconds(16);
  for(int8_t i = 0; i < sizeof pins; i ++){
    digitalWrite(pins[i], LOW);
  }
  delayMicroseconds(16);
  if (tick == 0) {
    for(int i = 0; i < sizeof pins; i ++){
      digitalWrite(pins[i], LOW);
    }
    while(true);
  }
}
