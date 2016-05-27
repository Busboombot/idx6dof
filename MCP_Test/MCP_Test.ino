#include <Wire.h>
#include <Adafruit_MCP23008.h>

Adafruit_MCP23008 mcp;

int limit_pins[6] =  {0,1,2}; // Limit switch inputs on the MCP23008
int limittrig_pins[6] =  {3,4,5}; // Limit switch trigger lights on the MCP23008
int limit_values[8];

#define N_STICKS 3

void setup() {
  Serial.begin(115200);
  mcp.begin(0);
  for (int i = 0; i < N_STICKS; i++){
    mcp.pinMode(limit_pins[i], INPUT);
    mcp.pullUp(limit_pins[i], HIGH);  // turn on a 100K pullup internally
    
    mcp.pinMode(limittrig_pins[i], OUTPUT);
  }
}

void loop() {
  for (int stick = 0; stick < N_STICKS; stick ++) {
    limit_values[stick] = mcp.digitalRead(limit_pins[stick]);
    mcp.digitalWrite(limittrig_pins[stick], HIGH);
    Serial.print(limit_values[stick]);
  }
  Serial.println("");
}
