
#include <Wire.h>
#include "Adafruit_MCP23008.h"

#include "digitalWriteFast.h"

uint8_t step_patterns[32][32] = { 
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // 0
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}, // 1
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}, // 2
{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1}, // 3
{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1}, // 4
{0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1}, // 5
{0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}, // 6
{0,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,1}, // 7
{0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1}, // 8
{0,0,0,1,0,0,0,1,0,0,1,0,0,0,1,0,0,1,0,0,0,1,0,0,1,0,0,0,1,0,0,1}, // 9
{0,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1}, // 10
{0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,1}, // 11
{0,0,1,0,0,1,0,1,0,0,1,0,0,1,0,1,0,0,1,0,0,1,0,1,0,0,1,0,0,1,0,1}, // 12
{0,0,1,0,1,0,0,1,0,1,0,0,1,0,1,0,0,1,0,1,0,0,1,0,1,0,0,1,0,1,0,1}, // 13
{0,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1}, // 14
{0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}, // 15
{0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}, // 16
{0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1}, // 17
{0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,1}, // 18
{0,1,0,1,0,1,1,0,1,0,1,1,0,1,0,1,1,0,1,0,1,1,0,1,0,1,1,0,1,0,1,1}, // 19
{0,1,0,1,1,0,1,1,0,1,0,1,1,0,1,1,0,1,0,1,1,0,1,1,0,1,0,1,1,0,1,1}, // 20
{0,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1}, // 21
{0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1,1}, // 22
{0,1,1,0,1,1,1,0,1,1,0,1,1,1,0,1,1,0,1,1,1,0,1,1,0,1,1,1,0,1,1,1}, // 23
{0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1}, // 24
{0,1,1,1,0,1,1,1,1,0,1,1,1,0,1,1,1,1,0,1,1,1,0,1,1,1,1,0,1,1,1,1}, // 25
{0,1,1,1,1,0,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,0,1,1,1,1,0,1,1,1,1,1}, // 26
{0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1}, // 27
{0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1}, // 28
{0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1}, // 29
{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, // 30
{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, // 31
};



Adafruit_MCP23008 mcp;

uint8_t dir;
uint8_t stp;
uint8_t stp_idx;

uint8_t stopStep[6] = {0,0,0};
uint8_t pstick_dir[6];

int8_t stick_val;
int analog_val;

int stp_idx_values[8];
uint8_t dir_values[8];
uint8_t limit_values[8];

int8_t  analog_pins[6] = {0,1,2};
int     analog_zero[6] = {0,0,0};
int8_t  step_pins[6] =   {2,4,6};
uint8_t dir_pins[6] =    {3,5,7};
int8_t  limit_pins[6] =  {0,1,2}; // Limit switch inputs on the MCP23008
int8_t  limittrig_pins[6] =  {3,4,5}; // Limit switch trigger lights on the MCP23008

#define STEP_PIN(stick) step_pins[stick]
#define DIR_PIN(stick) dir_pins[stick]
#define ANALOG_PIN(stick) analog_pins[stick]

#define N_STICKS 3


void setup()
{
  //Serial.begin(9600);

  for ( int stick = 0; stick < N_STICKS ; stick ++ ){
    pinModeFast(STEP_PIN(stick), OUTPUT);
    pinModeFast(DIR_PIN(stick), OUTPUT);
  }

  for ( int stick = 0; stick < N_STICKS ; stick ++ ){
    // Find the zero point for each of the sticks. This assumes that they are
    // spring-loaded to return to the middle and are not being moved during startup.
    
    analog_zero[stick] = analogRead( ANALOG_PIN(stick) ); 
  }    

  mcp.begin();      // use default address 0

  for (int i = 0; i < N_STICKS; i++){
    mcp.pinMode(limit_pins[i], INPUT);
    mcp.pullUp(limit_pins[i], HIGH);  // turn on a 100K pullup internally
    
    mcp.pinMode(limittrig_pins[i], OUTPUT);
  }
  
}




void loop()
{

  /* Read the values from the joysticks and compute the direction and 
   * which step pattern to use */
   
  for ( int stick = 0; stick < N_STICKS ; stick ++ ){
    limit_values[stick] = mcp.digitalRead(limit_pins[stick]); // Read first to interupt pots
    analog_val = analogRead( ANALOG_PIN(stick));
    
    // Map the analog values 0->1023 to -32 to 32
    stick_val = map(analog_val-analog_zero[stick], -512, 512,-31,31);
    
    
    if (stopStep[stick] == 0) {
      if (stick_val > 0) {
        dir = HIGH;
        stp_idx = stick_val;
        
      } else if (stick_val < 0) {
        dir = LOW;
        stp_idx = -stick_val;
        
      } else  {
        /* Force 0 to output nothing. */
        dir = LOW;
        stp_idx = 0;
      }
    }
    if (limit_values[stick] == LOW && stopStep[stick] < 2) { // Write after initial model to overwrite values
      stopStep[stick] = 1;
      if (stick_val > 0) {
        dir = LOW;
        stp_idx = 1;
      }
      else if (stick_val < 0) {
        dir = HIGH;
        stp_idx = 1;
      }
      else {
        dir = LOW;
        stp_idx = 0;
      }
      pstick_dir[stick] = dir;
    }
    if (stopStep[stick] == 1 && limit_values[stick] == HIGH) {
      stopStep[stick] = 2;
      dir = LOW;
      stp_idx = 0;
    }
    if (stopStep[stick] == 2) {
      if (pstick_dir[stick] == HIGH && stick_val > 0) {
        stopStep[stick] = 0;
      }
      if (pstick_dir[stick] == LOW && stick_val < 0) {
        stopStep[stick] = 0;
      }
    }

    stp_idx_values[stick] = stp_idx;
    dir_values[stick] = dir;
    
  }


  for (int n = 0; n < 32; n ++){ /* Iterate over all of the steps pattern slots */

    /* Iterate over the sticks inside to ensure that their steps are smoothly interleaved. */
    for ( int stick = 0; stick < N_STICKS ; stick ++ ){

      dir = dir_values[stick];
      digitalWriteFast2(DIR_PIN(stick),  dir);
      
      stp = step_patterns[stp_idx_values[stick]][n];
      digitalWriteFast2(STEP_PIN(stick),  stp ); 

      mcp.digitalWrite(limittrig_pins[stick], limit_values[stick]);

    }

    delayMicroseconds(16);

    /* Take the step low, since the motor driver needs a transition to acknowledge a pulse. */
    for ( int stick = 0; stick < N_STICKS ; stick ++ ){
      digitalWriteFast2(STEP_PIN(stick), LOW);
    }

    delayMicroseconds(16);
    

  }
}




