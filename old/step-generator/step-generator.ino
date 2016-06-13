// usbjoystick_test
// This sketch demonstrates simple use of the USBJoystick library
// It intialises the library, establishes callbacks for when inputs change
// and prints out details whenever an input changes and a callback is called.
//
// USB Host Shield uses an interrupt line, but does not establish an interrupt callback
// Note special requirements in documentation if you are using an older version of the USB Host Shield and a newer 
// version of the USB_Host_Shield library.
// Note special requirements to configure the WiShield librray to support the UDP application. 
// mikem@airspayce.com

#include <Usb.h>
#include <USBJoystick.h>

#include "digitalWriteFast.h"

// Our singleton joystick
USBJoystick joy;

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




uint8_t dir;
uint8_t stp;
uint8_t stp_idx;
int8_t v;

int stp_idx_values[4];
uint8_t dir_values[4];


/*int8_t step_pins[4] = {3,4,5,6};
uint8_t dir_pins[4] = {2,7,8,9};*/

int8_t step_pins[4] = {4,6,4,6};
uint8_t dir_pins[4] = {5,7,5,7};

#define STEP_PIN(stick) step_pins[stick]
#define DIR_PIN(stick) dir_pins[stick]

/*#define STEP_PIN(stick) (PIN0+stick*2)+1
#define DIR_PIN(stick) PIN0+stick*2*/

#define N_STICKS 2

void setup()
{
  /*Serial.begin(9600);*/
  
  joy.init();

  for ( int stick = 0; stick < N_STICKS ; stick ++ ){
    pinModeFast(step_pins[stick], OUTPUT);
    pinModeFast(step_pins[stick], OUTPUT);
  }

                                      
}

void debug_print(int n, int stick, int v, int dir, int stp_idx, int stp)
{
      Serial.print(n, DEC);
      Serial.print("\t");
      
      Serial.print(STEP_PIN(stick), DEC);
      Serial.print("\t");

      Serial.print(v, DEC);
      Serial.print("\t");

      Serial.print(dir, DEC);
      Serial.print("\t");

      Serial.print(stp_idx, DEC);
      Serial.print("\t");

      Serial.print(stp, DEC);
      Serial.print("\t");
      
      Serial.println("");
}


void loop()
{

  joy.run();

  /* Read the values from the joysticks and compute the direction and 
   * which step pattern to use */
  for ( int stick = 0; stick < N_STICKS ; stick ++ ){
    /* The raw value runs from 0 to 255, so shift it down to -128 to 127*/
    v = joy.stickValue(stick) - 128;

    uint8_t dir;
    uint8_t stp;
    
    if (v > 0) {
      dir = HIGH;
      stp_idx = (v>>2);
      
    } else if ( v < 0 ) {
      dir = LOW;
      stp_idx = ((-1*(v+1))>>2);
      
    } else  {
      /* Force 0 to output nothing. */
      dir = LOW;
      stp = LOW;
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

    }

    /*delayMicroseconds(50);*/

    /* Take the step low, since the motor driver needs a transition to acknowledge a pulse. */
    for ( int stick = 0; stick < N_STICKS ; stick ++ ){
      digitalWriteFast2(STEP_PIN(stick), LOW );
    }

    /*delayMicroseconds(50);*/
    

  }
}



