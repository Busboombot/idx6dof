
#define COUNTER_PIN_1 2
#define COUNTER_PIN_2 3

volatile int16_t count1 = 0;
volatile int16_t count2 = 0;

void setup() {
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN_1), count_pulse_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN_2), count_pulse_2, FALLING);

  Serial.begin(115200);

}

void loop() {

  delay(2000);

  Serial.print(count1); Serial.print(" ");
  Serial.print(count2); Serial.print(" ");
  Serial.print(count1-count2); Serial.print("\n");
  
}

void count_pulse_1(){
  count1 += 1;
}

void count_pulse_2(){
  count2 += 1;
}


