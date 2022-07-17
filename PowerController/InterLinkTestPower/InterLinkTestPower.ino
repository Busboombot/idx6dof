// Test the Power side of the interlink ( Mega, Master) 


#define ESTOP_PIN 41

#include <Wire.h>

void setup() {
  //Wire.begin(); // join i2c bus (address optional for master)
  Serial1.begin(9600);
  Serial.begin(9600);
  pinMode(ESTOP_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

long x = 0;

void loop() {
 
 // Wire.beginTransmission(8); // transmit to device #8
  //Wire.write("x is ");        // sends five bytes
  //Wire.write(x);              // sends one byte
  //Wire.endTransmission();    // stop transmitting
  x++;
    

  Serial1.print("x=");
  Serial1.println(x);


  Serial.print(digitalRead(ESTOP_PIN));
  Serial.print(' ');
  Serial.print("x=");
  Serial.println(x);

  digitalWrite(LED_BUILTIN, digitalRead(ESTOP_PIN));
  
  delay(500);
}
