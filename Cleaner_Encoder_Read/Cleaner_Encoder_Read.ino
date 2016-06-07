#include <Encoder.h>

#define NUM_LIMITS 6
#define NUM_ENC 3
#define SER_DEL 1000

struct ENC {
  
  int16_t p;
  int16_t prev_p;
  int8_t d;
  
};

ENC encOne;
ENC encTwo;
ENC encThree;

int8_t LimPins[NUM_LIMITS] = {A0, A1, A2, A3, A4, A5};
int16_t lim_vals[NUM_LIMITS];

uint16_t ser_clk;

Encoder ecOne(4, 5), ecTwo(6, 7), ecThree(8, 9);

void setup() {
  
  Serial.begin(115200);
  
}

void loop() {
  
  encOne.p = ecOne.read();
  encTwo.p = ecTwo.read();
  encThree.p = ecThree.read();

  if (encOne.p < encOne.prev_p) {
    encOne.d = 0;
  }
  else {
    encOne.d = 1;
  }
  if (encTwo.p < encTwo.prev_p) {
    encTwo.d = 0;
  }
  else {
    encTwo.d = 1;
  }
  if (encThree.p < encThree.prev_p) {
    encThree.d = 0;
  }
  else {
    encThree.d = 1;
  }
  encOne.prev_p = encOne.p;
  encTwo.prev_p = encTwo.p;
  encThree.prev_p = encThree.p;

  if (ser_clk > SER_DEL) {
    Serial.println('S');
    Serial.print((encOne.p>>8), BIN);
    Serial.println(encOne.p, BIN);
    Serial.println(encOne.d, BIN);
    Serial.print((encTwo.p>>8), BIN);
    Serial.println(encTwo.p, BIN);
    Serial.println(encTwo.d, BIN);
    Serial.print((encThree.p>>8), BIN);
    Serial.println(encThree.p, BIN);
    Serial.println(encThree.d, BIN);
    for (int i = 0; i < NUM_LIMITS; i++) {
      lim_vals[i] = analogRead(LimPins[i]);
      Serial.println(lim_vals[i], BIN);
    }
    Serial.println('E');
  }
  
  ser_clk++;  
}
