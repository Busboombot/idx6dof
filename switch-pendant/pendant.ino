

#include "idx_pendant.h"


IDXPendant pendant  = IDXPendant();

void setup() {
  pendant.begin();
  Serial.begin(9600);
}

char command_code[3];
int command_pos = 0;
bool command_complete = false;

bool auto_print = true;

void serialEvent() {

  char c;

  while (Serial.available()) {
    
    c = Serial.read();

    if(isalpha(c)){
      command_code[command_pos++]  = c; 
    }

    if (command_pos == 2){
      command_code[command_pos++]  = '\0';
      command_pos = 0;
      command_complete = true;
      
    }
  }
}

long lastTime = millis() - 500;

void loop() {
  if (pendant.run_once() && auto_print){
    
    Serial.println(pendant.outstr());
  }

  if (command_complete){

    Serial.println(command_code);

    // Display an identity string
    if (strcmp(command_code,"id") == 0){
      Serial.println("pendant");

    // print the output string
    } else if (strcmp(command_code,"pp") == 0){
      Serial.println(pendant.outstr());

    // Turn automatic printing on
    } else if (strcmp(command_code,"ao") == 0){
      auto_print = true;

    // turn automatic print off
    } else if (strcmp(command_code,"af") == 0){
      auto_print = false;
      
    }

    command_complete = false;
    command_pos = 0; 
  }
  

}
