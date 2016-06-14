

#define LED_PIN 13
#define DIRECTION_PIN 2
#define CHANGE_DIR_N 1000
byte direction = LOW;
byte pins[] = {3,4,5,6,7,8,9,10,11};
byte pins_val[sizeof pins] = {LOW};

void setup() {

  for( int i =0; i < sizeof pins; i++){
    
    digitalWrite(pins[i], LOW);
    pinMode(pins[i], OUTPUT);
  }

  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
}

int32_t tick = 1;
int32_t cycle = 0;

void loop() {

  tick++;
  cycle++;

  // change direction every N steps. N*2 b/c this block is called for 
  // every change, not just rising pulses. 
  if (cycle == CHANGE_DIR_N * 2){ 
    cycle = 0;
    direction = direction == LOW ? HIGH : LOW;
    digitalWrite(DIRECTION_PIN, direction);
    digitalWrite(LED_PIN, direction);
    delay(100000);
  }
  
  for(int i = 0; i < sizeof pins; i ++){

    if( tick % i == 0){
      pins_val[i] = pins_val[i] == LOW ? HIGH : LOW;
      digitalWrite(pins[i], pins_val[i]);
    }

    delayMicroseconds(1);
  }

}
