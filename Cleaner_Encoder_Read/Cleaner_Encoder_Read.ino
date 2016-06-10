#include <Encoder.h>

#define NUM_LIMITS 1
#define NUM_ENC 1
#define SER_DEL 1000
#define MAX_INDEXS 30
#define VEL_DELTA_T 100 //ms

struct ENC {
  
  int p;
  int prev_p;
  int8_t d;
  
};

ENC enc;

int8_t LimPin = A0;
int lim_val, p_lim_val;
int lim_low_begin, lim_low_end;
int lim_high_begin, lim_high_end;

int index_one, index_two;
int index_state;

int delta_pos;
int p_time;
int vel;

int ser_clk;

Encoder ecOne(3, 4);

void setup() {
  
  Serial.begin(115200);
  pinMode(LimPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), index, RISING);
  
}

void loop() {
  
  enc.p = ecOne.read();
  limit();
  velocity();
  
  if (enc.p < enc.prev_p) {
    enc.d = -1;
  }
  
  else {
    enc.d = 1;
  }
  
  enc.prev_p = enc.p;
  
  if (ser_clk > SER_DEL) {
    serialPrint();
  }
  
  ser_clk++;
}

void serialPrint() {
  String ser_data = (String)millis();
  ser_data += "|" + (String)enc.p;
  ser_data += "|" + (String)enc.d;
  ser_data += "|" + (String)lim_val;
  ser_data += "|" + (String)abs(index_one - index_two);
  ser_data += "|" + (String)abs(lim_high_end - lim_high_begin);
  ser_data += "|" + (String)abs(lim_low_end - lim_low_begin);
  ser_data += "|" + (String)vel; //Steps per ms
  Serial.println(ser_data);
}

void limit() {
  lim_val = digitalRead(LimPin);
  
  if (lim_val != p_lim_val) {
    if (lim_val == HIGH) {
      lim_low_end = enc.p;
      lim_high_begin = enc.p;
    }
    else {
      lim_high_end = enc.p;
      lim_low_begin = enc.p;
    }
  }
  
  p_lim_val = lim_val;
}

void index() {
  switch (index_state) {
    case 0:
      index_one = enc.p;
      index_state = 1;
    break;
    case 1:
      index_two = enc.p;
      index_state = 0;
    break;
  }
}

void velocity() {
  if (millis() >= p_time + VEL_DELTA_T) {
    p_time = millis();
    delta_pos = abs(enc.p - enc.prev_p);
    enc.prev_p = enc.p; // Set after taking delta pos
    vel = delta_pos/VEL_DELTA_T;
  }
}

