/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

#define NUM_ENC 3
#define SER_WRITE_INTER 100
#define VEL_READ_DEL 1000 // How often to calc velocity, in miliseconds

struct ENC {
  
  int32_t a;
  int32_t b;
  int32_t c;
  float va;
  float vb;
  float vc;
  
};

unsigned long milli_store;
int32_t last_velocity_calc = 0;
int32_t last_velocity_step = 0;
int32_t pos_store[3]; // last positions

float velocity_denominator = 0.0;

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

  // Pre-compute the denominator time-step for calculating velocity
  // converting it from milis to seconds
  velocity_denominator = float(VEL_READ_DEL) / 1000.0;
  
}

void loop() {

  enc.a = encOne.read(); // W/o PWM-202kHz | W/ PWM-202kHz | W/Inter-727kHz
  enc.b = encTwo.read();
  enc.c = encThree.read();
  
  if (clk++ == 200) {
    Serial.print(enc.a);Serial.print(" ");Serial.print(enc.va);Serial.print(", ");
    Serial.print(enc.b);Serial.print(" ");Serial.print(enc.vb);Serial.print(", ");
    Serial.print(enc.c);Serial.print(" ");Serial.print(enc.vc);Serial.print(", ");
    Serial.print("\n");
    clk = 0;
  }
  
  if (millis() > milli_store + VEL_READ_DEL){
    enc.va = float(enc.a-pos_store[0])/velocity_denominator;
    enc.vb = float(enc.b-pos_store[1])/velocity_denominator;
    enc.vc = float(enc.c-pos_store[2])/velocity_denominator;
    
    pos_store[0] = enc.a;
    pos_store[1] = enc.b;
    pos_store[2] = enc.c;
    milli_store = millis();
  }


  
}
