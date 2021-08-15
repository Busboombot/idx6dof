#include "Arduino.h"

/*
  Based on:
  Button.h - Library for Button Debounce.
  Created by Maykon L. Capellari, September 30, 2017.
  Released into the public domain.
*/


#define pt(x) Serial.print(x)
#define pl(x) Serial.println(x)

class Button {
  private:
    String _label;
    int _pin;
    unsigned long _delay;
    unsigned long _lastDebounceTime;
    int _lastStateBtn;
    bool _setState; // The  button state tht is considered "set"
    bool _wasSet; // Flagged when button is True
    bool _wasUnset; // Flagged when button is True

    bool isTimeToUpdate(){
      return (millis() - _lastDebounceTime) > _delay;
    }

  public:
    Button(String label, int pin, bool setState = true, unsigned long delay=250) : 
      _label(label), _pin(pin), _delay(delay), _setState(setState) {
      pinMode(pin, INPUT_PULLUP);
      _lastDebounceTime = 0;
      _lastStateBtn = HIGH;
    }

    String label(){
      return _label;
    }
 
   void update(){
      
      if(!isTimeToUpdate()) {
        return;
      }
    
      _lastDebounceTime = millis();
      int btnState = (digitalRead(_pin) == _setState);
      
      if(btnState == _lastStateBtn){
        return;
      }
      
      _lastStateBtn = btnState;

      if (btnState){
        _wasSet = true;
      } else {
        _wasUnset = true;
      }
      
      stateChanged();
      
   
    }

    int state(){
      return _lastStateBtn;
    }


    // Clear the _wasSet flag
    void clear(){
      _wasSet = false;
    }

    bool checkSetAndClear(){
      bool ws = _wasSet;
      _wasSet = false;
      return ws;
      
    }

    bool checkUnsetAndClear(){
      bool ws = _wasUnset;
      _wasUnset = false;
      return ws;
      
    }
    
    int stateChanged(){}
};

#define NBUTTONS 4
Button buttons[NBUTTONS] = {
  Button("reset",12, false),
  Button("key",11, false),
  Button("estop",10, false),
  Button("power",9, false)
};


class StateMachine {

  private: 
    static const int OFF_STATE = 10;

    void (StateMachine::*current_state)() = &StateMachine::offState;
   
    Button buttons[NBUTTONS] = {
      Button("reset",12, false),
      Button("key",11, false),
      Button("estop",10, false),
      Button("power",9, false)
    };

    bool checkReset(){ return buttons[0].checkSetAndClear(); }
    bool checkKeyOn(){ return buttons[1].state()==true; }
    bool checkKeyOff(){ return buttons[1].state()==false; }
    bool checkEStop(){ return buttons[2].checkSetAndClear(); }
    bool checkPower(){ return buttons[3].checkSetAndClear(); }

    bool checkAbort(){
      return checkKeyOff() | checkEStop();
    }
    
    void offState() {}
    void lowPowerOnState() {}
    void highPowerOnState() {}
    void errorState(){

      if (checkReset()){
        
      }
      
    }
    void shuttingDownState(){}

    // Transition to other states
    void toOffState() {
      current_state = &StateMachine::offState;
    }
    void toLowPowerOnState()  {
      current_state = &StateMachine::lowPowerOnState;
    }
    void toHighPowerOnState()  {
      current_state = &StateMachine::highPowerOnState;
    }
    void toErrorState() {
      current_state = &StateMachine::errorState;
    }
    void toShuttingDownState() {
      current_state = &StateMachine::shuttingDownState;
    }


    void turnOnLowPower(){}
    void turnOnHighPower(){}
    void turnOffLowPower(){}
    void turnOffHighPower(){}



    
  public:

    StateMachine(){
      
    }

    void update(){
      for(int i = 0; i < NBUTTONS; i++){
        buttons[i].update();
      }
   
    }
    


  
};

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  Serial.print(NBUTTONS);
  Serial.println(" buttons");

}

void loop() {
  for(int i = 0; i < NBUTTONS; i++){
    buttons[i].update();

    if (buttons[i].checkSetAndClear()){
      pt(buttons[i].state()); pt(' ');
      pl(buttons[i].label());
    }
  }

}
