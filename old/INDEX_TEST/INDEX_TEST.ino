#include <Encoder.h>

#define INDEX_PIN 4

uint32_t STPS;
int32_t prev_pos;
uint8_t bounce = 0;
uint8_t indx_in;

Encoder ecOne(2, 3);
void setup() {
  // put your setup code here, to run once:
  pinMode(INDEX_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  indx_in = digitalRead(INDEX_PIN);
  if (indx_in == 1 && bounce == 0) {
    index();
    bounce = 1;
  }
  else if (indx_in == 0 && bounce == 1) {
    bounce = 0;
  }
}-

void index() {
  Serial.println("INDEX");
  Serial.println(ecOne.read()-prev_pos);
  prev_pos = ecOne.read();
}

