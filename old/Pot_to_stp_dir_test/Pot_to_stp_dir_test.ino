int input;
int clk;
#define AN_DEL 500
#define STP_PIN 5
#define DIR_PIN 6

void setup() {
  pinMode(STP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}

void loop() {
  if (clk == 0) {
    input = map(analogRead(A0)-512, -512, 512, -52, 52);
    clk = AN_DEL;
    if (input > 0) {
      PORTD |= (1<<DIR_PIN);
    }
    else {
      PORTD &= (~(1<<DIR_PIN));
    }
  }
  clk--;
  
  PORTD |= (1<<STP_PIN);
  delayMicroseconds(abs(input));
  PORTD &= (~(1<<STP_PIN));
  delayMicroseconds(abs(input));
}
