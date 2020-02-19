
#include <Encoder.h>
#include <EasyButton.h>
#include <Arduino.h>
#include <U8g2lib.h>
//#include "digitalWriteFast.h"
#include "DigitalIO.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

Encoder myEnc(2,3);
char buffer[20];

#define N_SWITCHES 5
signed char switch_buffer[N_SWITCHES];
const uint8_t switch_pins[N_SWITCHES] = { 4,5,6,7,8 };
void set_state(int i, int state) {
  switch_buffer[i]--;
  if (switch_buffer[i] < -10){
    switch_buffer[i] == -10;
  }
}

bool get_state(int i) {
  return (bool)(switch_buffer[i] > 0);
}

void read_switches(){
  
  for(int i=0;i<N_SWITCHES;i++){
    switch_buffer[i] = digitalRead(switch_pins[i]);
  }
}

void print_switches(){
  for(int i=0;i<N_SWITCHES;i++){
    Serial.print( switch_buffer[i]);
  }
  Serial.println(' ');
}


#define DELAY_TIME 50
unsigned int lastTime = 0;
long lastPosition  = -999;
long lastPrintPos = -999;

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  u8g2.begin();
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  lastTime = millis();

  // Set all of the switch buffer switches to the off end of the range. 
  for(int i=0;i<N_SWITCHES;i++){
    switch_buffer[i] = -10;
    pinMode((uint8_t)switch_pins[i], INPUT_PULLUP);
  }

}

void loop() {
  read_switches();
  long newPosition = myEnc.read();
  if (newPosition != lastPosition) {
    lastPosition = newPosition;
    lastTime = millis();
  } else if (millis() - lastTime >= DELAY_TIME && lastPrintPos != lastPosition) {
    Serial.println(lastPosition);
    sprintf(buffer, "%d\0", lastPosition);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0, 20, buffer); // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display
    lastPrintPos = lastPosition;
    lastTime = millis();
    print_switches();
  }
}
