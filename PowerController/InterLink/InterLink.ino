// Test the interlink from the controller side. (slave )

#include <Wire.h>
#include <SoftwareSerial.h>

#define RX_PIN 2
#define TX_PIN 3
#define ESTOP_PIN 4

SoftwareSerial sserial(RX_PIN, RX_PIN);
String ssBuffer;
String hsBuffer;
bool eStop = false;
#define BUFFER_LENGTH 40

void setup()
{
  //Wire.begin(8);                // join i2c bus with address #8
  //Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output

  // Define pin modes for TX and RX
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(ESTOP_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  sserial.begin(9600);

  ssBuffer.reserve(BUFFER_LENGTH);
  hsBuffer.reserve(BUFFER_LENGTH);
}



bool readLineSS(SoftwareSerial& serial, String& str){

   while (serial.available()) { 

    char c = (char)serial.read();

    if (c == '\n'|| str.length() == BUFFER_LENGTH) {
      return true;
    } else if (c == '\r'){ // Ignore them
      //
    } else {
      str += c;
    }
  }
  return false;
}

bool readLineHS(HardwareSerial& serial, String& str){

   while (serial.available()) { 

    char c = (char)serial.read();

    if (c == '\n'|| str.length() == BUFFER_LENGTH) {
      return true;
    } else if (c == '\r'){ // Ignore them
      //
    } else {
      str += c;
    }
  }
  return false;
}

long i =0;
void loop()
{

  // Copy between the two serial ports. 
  if(readLineSS(sserial, ssBuffer)){
    Serial.println(ssBuffer);
    ssBuffer = "";
  }
  
  if(readLineHS(Serial, hsBuffer)){
    sserial.println(hsBuffer);
    if(hsBuffer == 'estop'){
      eStop = true;
    } else if (hsBuffer == 'reset'){
      eStop = false;
    }
    hsBuffer = "";
  }
  
  i++;

  digitalWrite(ESTOP_PIN, eStop);
  digitalWrite(LED_BUILTIN, eStop);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    //Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  //Serial.println(x);         // print the integer
}// Test the interlink from the controller side. (slave )

#include <Wire.h>
#include <SoftwareSerial.h>

#define RX_PIN 2
#define TX_PIN 3
#define ESTOP_PIN 4

SoftwareSerial sserial(RX_PIN, RX_PIN);
String ssBuffer;
String hsBuffer;
bool eStop = false;
#define BUFFER_LENGTH 40

void setup()
{
  //Wire.begin(8);                // join i2c bus with address #8
  //Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output

  // Define pin modes for TX and RX
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  pinMode(ESTOP_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  sserial.begin(9600);

  ssBuffer.reserve(BUFFER_LENGTH);
  hsBuffer.reserve(BUFFER_LENGTH);
}



bool readLineSS(SoftwareSerial& serial, String& str){

   while (serial.available()) { 

    char c = (char)serial.read();

    if (c == '\n'|| str.length() == BUFFER_LENGTH) {
      return true;
    } else if (c == '\r'){ // Ignore them
      //
    } else {
      str += c;
    }
  }
  return false;
}

bool readLineHS(HardwareSerial& serial, String& str){

   while (serial.available()) { 

    char c = (char)serial.read();

    if (c == '\n'|| str.length() == BUFFER_LENGTH) {
      return true;
    } else if (c == '\r'){ // Ignore them
      //
    } else {
      str += c;
    }
  }
  return false;
}

long i =0;
void loop()
{


  if(readLineSS(sserial, ssBuffer)){
    Serial.println(ssBuffer);
    ssBuffer = "";
  }
  
  if(readLineHS(Serial, hsBuffer)){
    sserial.println(hsBuffer);
    hsBuffer = "";
  }
  
  i++;

  digitalWrite(ESTOP_PIN, i&1);
  digitalWrite(LED_BUILTIN, i&1);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    //Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  //Serial.println(x);         // print the integer
}
