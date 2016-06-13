#define STEP_PIN 2
#define DIR_PIN 3

#define QUADA_PIN 4
#define QUADB_PIN 5

#define INDEX_PIN 6
#define LIMIT_PIN 7


uint16_t enc_pos;

#define ENC_STPS_PER_INDEX 8190
#define ENC_STPS_PER_ROT 180000
#define ENC_STPS_PER_LIM ENC_STPS_PER_ROT/2

#define fastSet(pin) (PORTD |= (1<<pin))
#define fastClear(pin) (PORTD &= (~(1<<pin)))

void setup() {
//  Serial.begin(115200);
  pinMode(STEP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STEP_PIN), stepIn, RISING);
  pinMode(DIR_PIN, INPUT);
  pinMode(QUADA_PIN, OUTPUT);
  pinMode(QUADB_PIN, OUTPUT);
  pinMode(INDEX_PIN, OUTPUT);
  pinMode(LIMIT_PIN, OUTPUT);
}

void loop() {
  
}

void stepIn() {
  if (((PIND & (1<<DIR_PIN))>>3) == 1) {
    enc_pos ++;
  }
  else {
    enc_pos --;
  }
  switch (enc_pos % 4) {
    case 0: // A=0, B=0
      fastClear(QUADA_PIN);
      fastClear(QUADB_PIN);
      if ((enc_pos % ENC_STPS_PER_INDEX) == 0) {
        fastSet(INDEX_PIN);
      }
      else {
        fastClear(INDEX_PIN);
      }
      break;
    case 1: // A=1, B=0
      fastSet(QUADA_PIN);
      fastClear(QUADB_PIN);
      if ((enc_pos % ENC_STPS_PER_INDEX) == 0) {
        fastSet(INDEX_PIN);
      }
      else {
        fastClear(INDEX_PIN);
      }
      break;
    case 2: // A=1, B=1
      fastSet(QUADA_PIN);
      fastSet(QUADB_PIN);
      if ((enc_pos % ENC_STPS_PER_INDEX) == 0) {
        fastSet(INDEX_PIN);
      }
      else {
        fastClear(INDEX_PIN);
      }
      break;
    case 3: // A=0, B=1
      fastClear(QUADA_PIN);
      fastSet(QUADB_PIN);
      if ((enc_pos % ENC_STPS_PER_INDEX) == 0) {
        fastSet(INDEX_PIN);
      }
      else {
        fastClear(INDEX_PIN);
      }
      break;
  }
  if ((enc_pos % ENC_STPS_PER_ROT) < ENC_STPS_PER_LIM) {
    fastSet(LIMIT_PIN);
  }
  else {
    fastClear(LIMIT_PIN);
  }
}

