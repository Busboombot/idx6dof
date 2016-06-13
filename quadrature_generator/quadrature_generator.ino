/*
 * Generate quadrature encoder, index and limit siggnals like one axis of an IXD 6 Axis robot. 
 * 
 * The program reads a potentiometer on analog A0 to set the step generation speed. 
 * Pin 4: Limit switch. High for half of axis rotation, low for the other half
 * Pin 5: Quadrature A
 * Pin 6: Quadrature B
 * Pin 7: Index
 */
 
#include <bithacks.h>
#include <digitalWriteFast.h>

int16_t analogVal;
int32_t next_analog_read = 0;

int8_t dir = 0;
uint16_t delay_time = 0;


uint8_t state = 0;
uint32_t enc_pos = 0;

uint8_t index_state = 0;

#define POT_PIN A0


#define ENC_STPS_PER_ROT 20000 
#define ENC_STPS_PER_INDEX 5000

#define MAX_DEL 250
#define MIN_DEL 0

void setup() {
  DDRD = DDRD | B11110000; // Set pins 4, 5, 6, and 7 as outputs
  Serial.begin(115200);
}

void loop() {
  
  if (micros() > next_analog_read ) {
      analogVal = analogRead(POT_PIN); // 109.5 us - Replace with 555 timer ADC?
      delay_time = map( abs(analogVal-512), 0, 512, MAX_DEL, 0);

      if (delay_time < 5) {
        delay_time = 0;
      }
      
      dir = (analogVal > 0) ? 1 : -1;

      next_analog_read = micros() + 1000;

  }

  delay(delay_time);
  
  enc_pos += dir;

  // Handle limit switch and encoder position value wrap-around
  if (enc_pos % ENC_STPS_PER_ROT == 0) {
    PORTD |=   B00010000; // Turn limit on
  
  } else if (enc_pos % (ENC_STPS_PER_ROT/2)  == 0) {
    PORTD &= (~B00010000); // turn limit off
    
  }

  if ((enc_pos % ENC_STPS_PER_INDEX) == 0) {
        PORTD |= B10000000; // turn Index on
  } else if ((enc_pos % ENC_STPS_PER_INDEX) == 3) {
        PORTD &= (~B10000000); // turn off index
  }

  switch (enc_pos % 4) {
    case 0:
      PORTD |=   B01000000;  // 6 = HIGH
      PORTD &= (~B00100000); // 5 = LOW
      state = 1;
      break;
    case 1:
      PORTD |=   B01100000; // 6 = HIGH|5 = HIGH
      state = 2;
      break;
    case 2:
      PORTD &= (~B01000000); // 6 = LOW
      PORTD |=   B00100000; // 5 = HIGH
      state = 3;
      break;
    case 3:
      PORTD &= (~B01100000); // 6 = LOW|5 = LOW
      state = 0;
      break;
  }
}



