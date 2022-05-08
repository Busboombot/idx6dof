/* Encoder Library - Basic Example
   http://www.pjrc.com/teensy/td_libs_Encoder.html

   This example code is in the public domain.
*/


#include <Encoder.h>
#include <EasyButton.h>
#include <Arduino.h>
#include <Wire.h>

//#include "digitalWriteFast.h"
#include "DigitalIO.h"

byte LED_PIN = 5;
byte ESTOP_PIN = 18;// A4
byte ENSW_PIN = 19; // A6 Enable switch on side

Encoder myEnc(3,2);

const uint8_t axis_switch_pins[] = { 9,8,7,6 };
#define N_AXIS_SWITCHES (sizeof(axis_switch_pins)/sizeof(*axis_switch_pins))
unsigned int last_axis_switch_time = 0;
int last_axis_candidate = 0;
int last_axis_switch = 0;

const uint8_t speed_switch_pins[] = { 10,12,11 };
#define N_SPEED_SWITCHES (sizeof(speed_switch_pins)/sizeof(*speed_switch_pins))
unsigned int last_speed_switch_time = 0;
int last_speed_candidate = 0;
int last_speed_switch = 0;

const uint8_t speeds[] = { 0, 1, 10, 100 };
const char axes[] = { 'N', 'X', 'Y', 'Z', 'W', 'O', 'A', 'B', 'C', 'D' };

#define DEBOUNCE_DELAY 100

uint8_t read_axis_switches(){
  // Read switches, with debouncing. 

  // What is the highest switch value that is currently set, 
  // among the axis switches?
  int max_switch = 0;
  for(int i=0;i<N_AXIS_SWITCHES;i++){
    if (!digitalRead(axis_switch_pins[i])) {
      max_switch = i+1;
    }
  }

  if ( max_switch != last_axis_switch){ // Current value is different that what was last reported
    if ( max_switch != last_axis_candidate){ // Current value is also different than the new value we are testing. 
      // Reset the candidate timer
      last_axis_candidate = max_switch;
      last_axis_switch_time = millis();
    } else if (millis() - last_axis_switch_time >= DEBOUNCE_DELAY){ /// Candidate timer expired
      last_axis_switch =  last_axis_candidate;    
    }
  } 
  return last_axis_switch;
}



uint8_t read_speed_switches(){
  // Read switches, with debouncing. 

  // What is the highest switch value that is currently set, 
  // among the speed switches?
  int max_switch = 0;
  for(int i=0;i<N_SPEED_SWITCHES;i++){
    if (!digitalRead(speed_switch_pins[i])) {
      max_switch = i+1;
    }
  }

  if ( max_switch != last_speed_switch){ // Current value is different that what was last reported
    if ( max_switch != last_speed_candidate){ // Current value is also different than the new value we are testing. 
      // Reset the candidate timer
      last_speed_candidate = max_switch;
      last_speed_switch_time = millis();
    } else if (millis() - last_speed_switch_time >= DEBOUNCE_DELAY){ /// Candidate timer expired
      last_speed_switch =  last_speed_candidate;    
    }
  } 

  return last_speed_switch;
}

unsigned int lastTime = 0;
int32_t positions[sizeof(axes)] = {0};
int32_t last_pos  = 0;

EasyButton enable_button(ENSW_PIN);
EasyButton estop_button(ESTOP_PIN);

void setup() {
  Serial.begin(9600);
  
  //Wire.begin(); // join i2c bus (address optional for master)
  
  lastTime = millis();

  // Set all of the switch buffer switches to the off end of the range. 
  for(int i=0;i<N_AXIS_SWITCHES;i++){
    pinMode((uint8_t)axis_switch_pins[i], INPUT_PULLUP);
  }

    for(int i=0;i<N_SPEED_SWITCHES;i++){
    pinMode((uint8_t)speed_switch_pins[i], INPUT_PULLUP);
  }

  pinMode(LED_PIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(ENSW_PIN, INPUT_PULLUP);

}

int last_switch = 0;
bool led_state = false;
uint16_t v = 0;

void loop() {

  bool ensw = !enable_button.read(); // Enable Switch, selects axis
 
  uint8_t axis_sw = read_axis_switches();
  uint8_t spd_switch = read_speed_switches();
  uint8_t spd=speeds[spd_switch];

  bool estop = estop_button.read();
  
  int this_switch = spd_switch*1000 + estop*100+ensw*10+axis_sw; // Keep track of changes to both switches

  uint8_t axis = axis_sw + ( ((int)!ensw)*5 );
  
  int32_t current_pos = myEnc.read(); 

  current_pos /= 4; // The encoder increments by 4 each step
  
  if (current_pos != last_pos or this_switch != last_switch ) {

    led_state = !led_state;

    positions[axis] += (current_pos - last_pos)*spd;

    Serial.print(estop ? "-" : "S");
    Serial.print(ensw ? "-" : "E");
    Serial.print(spd_switch);
    Serial.print(axis+1);
    Serial.print(" ");
    Serial.print(axes[axis]);
    Serial.println(positions[axis]);

    //Wire.beginTransmission(8); // transmit to device #8
    //Wire.write(axes[axis]);
    //Wire.write((char*)&positions[axis], 4);
    //Wire.endTransmission();

    last_pos = current_pos;
    last_switch = this_switch;

    digitalWrite(LED_PIN, led_state);

  }
  
}
