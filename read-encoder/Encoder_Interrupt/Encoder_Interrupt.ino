#include <Encoder.h>
#include <PacketSerial.h>


IntervalTimer clearTimer; // Timer for clearing the LIMIT_INTR pin
IntervalTimer clearLedTimer; // Timer for clearing the LED

#define LIMIT_PIN 4
#define LIMIT_INTR_PIN 33

PacketSerial ps;

long last_time = millis();

volatile uint8_t limit_state = 0;
volatile int limit_pos = 0;
volatile int limits_changed = 0; // Number of limits that have changed since last limit interrupt

int auto_print = false; // Automatically print limits and positions.

typedef void (*voidfp)();
void limit_change(int n);

typedef struct {
  uint8_t limit_states[6] = {0};
  uint16_t pad1;
  int32_t positions[6] = {0};
} EncoderReport;

EncoderReport encoder_report;

enum limit {
  HL = 0b10,
  LH = 0b01,
  HH = 0b11,
  LL = 0b00
};

class LimitEncoder {

  private:
    int axis_n; // Axis Number
    int pin_a; // Encoder A
    int pin_b; // Encoder B
    int pin_i; // Encoder Index
    int pin_l; // Limit pin


    long last_limit_pos; // Position at last limit changed
    long pen_limit_pos; // Position at penultimate limit change
  
    int last_limit; // Most recent limit state after a change

    long last_pos;

    uint8_t limit_state;

    int dir;
    
    Encoder encoder;

  public:

    LimitEncoder(int axis_n, int pin_a, int pin_b, int pin_i, int pin_l):
      axis_n(axis_n), pin_a(pin_a), pin_b(pin_b), pin_i(pin_i), pin_l(pin_l), encoder(pin_a, pin_b) {
      //
    }

    // Attach the interrupt handler to the limit pin
    void attach(voidfp funct) {
      attachInterrupt(digitalPinToInterrupt(pin_l), funct, CHANGE);
    }

    int getAxisNumber() {
      return axis_n;
    }

    long getPosition() {
      last_pos = encoder.read();
      return last_pos;
    }

    long getLastPosition() {
      return last_pos;
    }

    int calcDirection(){
      long l = getLastPosition();
      long d = getPosition()-l; // distance changed
      d = (int)(d/abs(d)); // -1 or 1
      if(d != 0){
        dir = (d+1)/2; // 0 or 1
      }
      return dir;
    }

    int getDirection(){
      return dir;
    }
    
    void setPosition(long p) {
      encoder.write(p);
    }

    int getLimit() {
      return digitalRead(pin_l);
    }

    int getLastLimit() {
      return last_limit;
    }

    void limitChanged() {
      pen_limit_pos = last_limit_pos;
      last_limit_pos = getPosition();
 
      last_limit = getLimit();

      if(last_limit){
        if(dir){
          limit_state = LH;
        } else {
          limit_state = HL;
        }
        
      } else {
        if(dir){
          limit_state = HL;
        } else {
          limit_state = LH;
        }
        
      }
    }

    void limitUnChanged(){

      if(last_limit){
        limit_state = HH;
      } else {
        limit_state = LL;
      }
      
    }

    int getLimitState(){
      return limit_state;
    }
 
    long getLimitDiff() {
      return last_limit_pos - pen_limit_pos;
    }

    long getLastLimitPos() {
      return last_limit;
    }

};

LimitEncoder encoders[] = {
  LimitEncoder(0, 2, 3, 4, 5),
  LimitEncoder(1, 6, 7, 8, 9),
  LimitEncoder(2, 10, 11, 12, 24),
  LimitEncoder(3, 13, 14, 15, 16),
  LimitEncoder(4, 17, 18, 19, 20),
  LimitEncoder(5, 25, 26, 27, 28)

};

int n_encoders = sizeof encoders / sizeof encoders[0];


void updateReport(){
    for ( int i = 0; i < n_encoders; i++) {
      LimitEncoder& enc = encoders[i];

      encoder_report.limit_states[i] = enc.getLimitState() | (enc.getDirection() <<2 ) ;
      encoder_report.positions[i] = enc.getPosition();
    
  }
}

void limit_change(int n) {
  limits_changed += 1;
  
  for ( int i = 0; i < n_encoders; i++) {
    if (i == n) {
      encoders[i].limitChanged();
    } else {
      encoders[i].limitUnChanged();
    }
    
  }
  
}

void limit_change_1() {
  limit_change(0);
}
void limit_change_2() {
  limit_change(1);
}
void limit_change_3() {
  limit_change(2);
}
void limit_change_4() {
  limit_change(3);
}
void limit_change_5() {
  limit_change(4);
}
void limit_change_6() {
  limit_change(5);
}


voidfp limit_changes[] = {
  limit_change_1,
  limit_change_2,
  limit_change_3,
  limit_change_4,
  limit_change_5,
  limit_change_6
};

int n_limit_changes = sizeof limit_changes / sizeof limit_changes[0];


void setup() {

  ps.begin(115200);


  limit_state = (digitalRead(LIMIT_PIN) << 1);

  pinMode(LIMIT_INTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Attach the interrupts for limit changes
  for ( int i = 0; i < n_encoders; i++) {
    LimitEncoder& enc = encoders[i];
    enc.attach(limit_changes[enc.getAxisNumber()]);
  }
}

void loop() {

  if (Serial.available() > 0) {
    // read the incoming byte:
    char cmd = Serial.read();

    switch (cmd) {
      case 'c': // Clear
        for ( int i = 0; i < n_encoders; i++) {
          encoders[i].setPosition(0);
        }
        break;
      case 'r': // Report all positions
        Serial.print('r');
        for ( int i = 0; i < n_encoders; i++) {
          Serial.write(encoders[i].getPosition());
        }
        Serial.print('\n');
        break;
      case 'l': // Report last limit positions
        Serial.print('l');
        for ( int i = 0; i < n_encoders; i++) {
          Serial.write(encoders[i].getLastLimitPos());
        }
        Serial.print('\n');
        break;
      case 'a': // Auto print limits and positions.
        auto_print = true;
        break;
      case 'A': // Turn off auto print
        auto_print = false;
        break;
    }
  }

  for ( int i = 0; i < n_encoders; i++) {
    encoders[i].calcDirection();
  }


  if (limits_changed != 0) {
    limits_changed = 0;
    updateReport();
    ps.send((const uint8_t*)&encoder_report, sizeof(encoder_report));


    
  }

}