#include <bithacks.h>
#include <digitalWriteFast.h>

int16_t analogVal;

int8_t dir;
uint16_t del;
uint32_t del_clk;

uint8_t state;
uint32_t enc_pos;

uint8_t index_state = 0;

#define POT_PIN A0
#define QUAD_A 5
#define QUAD_B 6
#define INDEX 7
#define LIMIT 4

#define STPS_PER_ROT 2000 // Should be evenly divisable by 4
#define ENC_STPS_PER_ROT (STPS_PER_ROT * 80)
#define STPS_PER_INDEX 200
#define ENC_STPS_PER_INDEX (STPS_PER_INDEX * 80)

#define MAX_DEL 2000
#define MIN_DEL 0

void setup() {
  DDRD = DDRD | B11110000; // Set pins 4, 5, 6, and 7 as outputs
}

void loop() {
  
  if (micros() >= del_clk+del) {
    del_clk = micros();
    readAnalog();
    enc_pos += dir;
    if (enc_pos == ENC_STPS_PER_ROT) {
      enc_pos = 0;
    }
    else if (enc_pos == 0) {
      enc_pos = ENC_STPS_PER_ROT;
    }
    if ((enc_pos % ENC_STPS_PER_INDEX) == 0) {
      index_state = 1;
    }
    if (index_state != 0) {
      index();
    }
    encode();
    limit();
  }
}

void index() {
  switch (index_state) {
    case 1:
      PORTD |= B10000000;
    break;
    case 3:
      PORTD &= (~B10000000);
    break;
  }
  index_state++;
  if (index_state == 4) {
    index_state = 0;
  }
}

void encode() {
  state = enc_pos % 4;
  switch (state) {
    case 0:
      PORTD |= B01000000; // 6 = HIGH|5 = LOW
      PORTD &= (~B00100000);
    break;
    case 1:
      PORTD |= B01100000; // 6 = HIGH|5 = HIGH
    break;
    case 2:
      PORTD &= (~B01000000);
      PORTD |= B00100000; // 6 = LOW|5 = HIGH
    break;
    case 3:
      PORTD &= (~B01100000); // 6 = LOW|5 = LOW
    break;
  }
}

void limit() {
  if (enc_pos > ENC_STPS_PER_ROT/2) {
    PORTD |= B00010000;
  }
  else {
    PORTD &= (~B00010000);
  }
}

void readAnalog() {
  analogVal = analogRead(POT_PIN); // 109.5 us - Replace with 555 timer ADC?
  if (analogVal>512) {
    del = map(analogVal, 513, 1024, MAX_DEL, MIN_DEL);
    dir = 1;
  }
  else {
    del = map(analogVal, 0, 512, MIN_DEL, MAX_DEL);
    dir = -1;
  }
}

