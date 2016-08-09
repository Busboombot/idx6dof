#define QUAD_A 2
#define QUAD_B 3

volatile uint16_t encCount;

volatile uint8_t quadState = 0;

int8_t quadDir;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(QUAD_A), QuadPulseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_B), QuadPulseB, CHANGE);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(encCount);
  delay(500);
}

void QuadPulseA() {
  encCount--;
  if (quadState == 1 && digitalRead(QUAD_A) == HIGH) {
    quadDir = 0;
  }
  if (digitalRead(QUAD_A) == HIGH) {
    quadState = 0;
  }
}

void QuadPulseB() {
  encCount++;
  if (quadState == 0 && digitalRead(QUAD_B) == HIGH) {
    quadDir = 1;
  }
  if (digitalRead(QUAD_B) == HIGH) {
    quadState = 1;
  }
}
