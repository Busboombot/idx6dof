/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached

IntervalTimer clearTimer;

#define LIMIT_PIN 4
#define LIMIT_INTR_PIN 33


long oldPosition  = -999;
long last_time = millis();

volatile uint8_t limit_state = 0;
volatile int limit_pos = 0;
volatile int limits_changed = 0; // Number of limits that have changed since last limit interrupt
int dir  = 0;

typedef void (*voidfp)();

void limit_change(int n);


class LimitEncoder {

  private:
    int axis_n; // Axis Number
    int pin_a;
    int pin_b;
    int pin_l; // Limit pin

    void limit_isr(){
      
    }

  public:

    LimitEncoder(int axis_n, int pinA, int pinB, int pinL): axis_n(axis_n), pin_a(pinA), pin_b(pinB), pin_l(pinL){
      //
    }   

    void attach(voidfp funct){
      attachInterrupt(digitalPinToInterrupt(pin_l), funct, CHANGE);
    }

    
};

LimitEncoder encoders[] = {
  LimitEncoder(1,2,3,4)
};

int n_encoders = sizeof encoders / sizeof encoders[0];


void limit_change(int n){

  limits_changed += 1;

  //LimitEncoder& enc = encoders[n];
   
}

void limit_change_1(){ limit_change(1); }
void limit_change_2(){ limit_change(2); }
void limit_change_3(){ limit_change(3); }
void limit_change_4(){ limit_change(4); }
void limit_change_5(){ limit_change(5); }
void limit_change_6(){ limit_change(6); }


voidfp limit_changes[] = {
  limit_change_1,
  limit_change_2,
  limit_change_3,
  limit_change_4,
  limit_change_5,
  limit_change_6
};

int n_limit_changes = sizeof limit_changes / sizeof limit_changes[0];


void clearISR(){

  // digitalWrite(LIMIT_INTR_PIN, true);
  // clearTimer.begin(clearISR,2); a
  
  digitalWrite(LIMIT_INTR_PIN, false);
  clearTimer.end(); 
}


  
void setup() {
  Serial.begin(9600);
  
  limit_state = (digitalRead(LIMIT_PIN)<<1);

  pinMode(LIMIT_INTR_PIN, OUTPUT);

  for( int i; i < n_encoders; i++){
    
  }
  
}

void loop() {
  long newPosition = myEnc.read();

  
  if (newPosition > oldPosition){
    dir = 1;
  } else if  (newPosition < oldPosition){
    dir = -1;
  } 
  
  oldPosition = newPosition;
  
  if ( millis() - last_time >= 100){
    last_time = millis();
    
    //Serial.print(dir*10000);
    //Serial.print(" ");
    //Serial.print(((limit_state-1)*2-1)*10000);
    //Serial.print(" ");
    Serial.print(limit_pos*1200);
    Serial.print(" ");
    Serial.println(newPosition);

  }
}
