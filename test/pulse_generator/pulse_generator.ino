

#define DIRECTION_PIN 2
#define CHANGE_DIR_N 1000
byte direction = LOW;
byte pins[] = {3,4,5,6,7,8,9,10,11};
byte pins_val[sizeof pins] = {LOW};

void setup() {

  for( int i =0; i < sizeof pins; i++){
    pinMode(i, OUTPUT);
  }

  pinMode(DIRECTION_PIN, OUTPUT);
  
}

int32_t tick = 0;
int32_t cycle = 0;

void loop() {

  tick++;
  cycle++;

  if (cycle == CHANGE_DIR_N){ // change direction ever N steps
    cycle = 0;
    direction = direction == LOW ? HIGH : LOW;
    digitalWrite(DIRECTION_PIN, direction);
  }
  
  for(int i = 0; i < sizeof pins; i ++){

    if( tick % (i+1) == 0){
      pins_val[i] = pins_val[i] == LOW ? HIGH : LOW;
      digitalWrite(pins[i], pins_val[i]);
    }

    delayMicroseconds(1);
  }

}
