#include <digitalWriteFast.h>

int8_t pins[] = {3,5};
int32_t n_pulses[sizeof pins] = {100071, 204400};
int32_t pin_vals[sizeof pins];
void setup() {

  for(int i =0; i < sizeof pins; i++){
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

int8_t i;
void loop() {

  i = random(0, (sizeof pins)+1);
  if (pin_vals[i] < n_pulses[i]) {
    digitalWriteFast2(pins[i], HIGH);
    delayMicroseconds(4);
    digitalWriteFast2(pins[i], LOW);
    delayMicroseconds(4);
    pin_vals[i]++;
  }
}
