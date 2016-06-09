#include <Encoder.h>

#define LIM_PIN A0

uint32_t STPS;
int32_t prev_pos;
uint8_t bounce;
uint8_t lim_in, prev_lim_in;

Encoder ecOne(2, 3);
void setup() {
  // put your setup code here, to run once:
  pinMode(LIM_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  lim_in = digitalRead(LIM_PIN);
  if (lim_in != prev_lim_in && bounce == 0) {
    limit();
    bounce - 1;
  }
  else if (lim_in == prev_lim_in && bounce == 1) {
    bounce = 0;
  }
  prev_lim_in = lim_in;
}

void limit() {
  Serial.println("INDEX");
  Serial.println(ecOne.read()-prev_pos);
  prev_pos = ecOne.read();
}

