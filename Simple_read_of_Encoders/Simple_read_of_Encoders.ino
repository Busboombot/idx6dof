/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

#define NUM_ENC 3
#define SER_WRITE_INTER 100
#define VEL_READ_DEL 5000

struct ENC {
  
  int16_t a;
  int16_t b;
  int16_t c;
  int8_t va;
  int8_t vb;
  int8_t vc;
  
};

unsigned long milli_store;
int16_t pos_store[3];
uint16_t vel_clk;

ENC enc;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability- Reads every 1.25 us
//   Good Performance: only the first pin has interrupt capability - Reads every 4.92 us
//   Low Performance:  neither pin has interrupt capability- Reads every 4.92 us
Encoder encOne(4, 5), encTwo(6, 7), encThree(8, 9);
//   avoid using pins with LEDs attached
uint8_t clk;


void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {
//  PORTD = B00010000;
  enc.a = encOne.read(); // W/o PWM-202kHz | W/ PWM-202kHz | W/Inter-727kHz
  enc.b = encTwo.read();
  enc.c = encThree.read();
  
  if (clk == 200) {
    Serial.println(enc.a);
    Serial.println(enc.b);
    Serial.println(enc.c);
    Serial.println("|||");
    Serial.println(enc.va);
    Serial.println(enc.vb);
    Serial.println(enc.vc);
    Serial.println("---");
    clk = 0;
  }
  if (millis() >= milli_store + VEL_READ_DEL) {
    enc.va = (enc.a-pos_store[0])/VEL_READ_DEL;
    enc.vb = (enc.b-pos_store[1])/VEL_READ_DEL;
    enc.vc = (enc.c-pos_store[2])/VEL_READ_DEL;
    milli_store = millis();
    pos_store[0] = enc.a;
    pos_store[1] = enc.b;
    pos_store[2] = enc.c;
  }
  
  clk++;
  vel_clk++;
//  PORTD = B00000000;
  
}
