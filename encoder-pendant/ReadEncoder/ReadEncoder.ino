
#include <Encoder.h>
#include <Arduino.h>
#include <Wire.h>

Encoder encoder(4,5);

void setup() {
  Serial.begin(9600);
}

int print_counter = 100;
int32_t last_pos = 0;

void loop() {
 
  int32_t current_pos = encoder.read(); 

  if (current_pos != last_pos){
    
    if(print_counter -- == 0){
      Serial.println(current_pos);
      print_counter = 500;
    }
    last_pos = current_pos;
  }

}
