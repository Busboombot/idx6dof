/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */

#define NUM_OUT_PINS 8
#define NUM_IN_PINS 6
int outpins[] = { 23, 25, 27, 29, 31, 33, 34, 35 };
int inpins[] =  { 22, 24, 26, 28, 30, 32 };
int sw_no;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  for (int i = 0 ; i < NUM_OUT_PINS; i++){
    pinMode(outpins[i], OUTPUT);
  }

   for (int j = 0 ; j < NUM_IN_PINS; j++){
    pinMode(inpins[j], INPUT);
  }

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
}

char * code;

// the loop function runs over and over again forever
void loop() {

  for (int i = 0 ; i < NUM_OUT_PINS; i++){
    delay(5); 
    digitalWrite(outpins[i], HIGH);  
    for (int j = 0 ; j < NUM_IN_PINS; j++){
        delay(1);
        if (digitalRead(inpins[j])){
          sw_no = i+j*8;
          Serial.print( ' ' );
          Serial.print(sw_no, DEC);
          
        }
    }
    
    digitalWrite(outpins[i], LOW);
       
  }
  Serial.print('\n');

}
