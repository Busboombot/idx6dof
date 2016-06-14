
#define COUNTER_PIN_1 2
#define COUNTER_PIN_2 3

volatile int32_t count1 = 0;
volatile int32_t count2 = 0;

void setup() {
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN_1), count_pulse_1, RISING);
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN_2), count_pulse_2, RISING);

  Serial.begin(115200);

}

void loop() {

  delay(1000);

  Serial.print(count1); Serial.print(" ");
  Serial.print(count2); Serial.print("\n");
  
}

void count_pulse_1(){
  count1++;
}

void count_pulse_2(){
  count2++;
}


