#include "Arduino.h"

/*
  Based on:
  Button.h - Library for Button Debounce.
  Created by Maykon L. Capellari, September 30, 2017.
  Released into the public domain.
*/

#include "U8glib.h"
#include <SoftwareSerial.h>  


#define NBUTTONS 4

#define pt(x) Serial.print(x)
#define pl(x) Serial.println(x)
#define ps Serial.print(' ')


class FadeLed {

  private:
    int _pin;

    int ledState = LOW;             // ledState used to set the LED
  
    unsigned long lastBlinkTime = 0;        // will store last time LED was updated
    unsigned long lastFadeTime = 0;        // will store last time LED was updated
    

    const static int DEFAULT_BLINK_INT = 1000;   
    const static int DEFAULT_FADE_INT = 50; 
    const static int DEFAULT_FADE_SPEED = 5; 

    int blinkInterval = DEFAULT_BLINK_INT;
    int fadeInterval = DEFAULT_FADE_INT;
    int fadeAmount = DEFAULT_FADE_SPEED;    // how many points to fade the LED by
    int fadeMax = 255;

    
    int brightness = 0;    // how bright the LED is 

  public:
    int mode = FadeLed::BLINK;

  public:

  const static int ON = 1;
  const static int OFF = 2;
  const static int BLINK = 3;
  const static int FADE = 4; 

  FadeLed(int pin) :_pin(pin){
    pinMode(_pin, OUTPUT);
    
  }

  void setFade(int speed=DEFAULT_FADE_SPEED, int max=255){
    mode = FadeLed::FADE;
    fadeAmount = speed;
   
    fadeMax = max;
  }

  void setFadeInterval(int intv){
    fadeInterval = intv;
  }

  void setBlink(int interval=DEFAULT_BLINK_INT){
    blinkInterval = interval;
    mode = FadeLed::BLINK;
  }

  void setOn(){
    mode = FadeLed::ON;
  }

  void setOff(){
    mode = FadeLed::OFF;
  }
  
  void update(){


    switch (mode){
      case  FadeLed::ON: {
        digitalWrite(_pin, HIGH);
        break;
      }
      case  FadeLed::OFF: {
        digitalWrite(_pin, LOW);
        break;
      }
      case  FadeLed::FADE: {
        if (millis() - lastFadeTime >= fadeInterval) {
          // save the last time you blinked the LED
          lastFadeTime = millis();
    
          brightness = brightness + fadeAmount;
      
          if (brightness <= 0 || brightness >= fadeMax) {
            fadeAmount = -fadeAmount;
            if (brightness <= 0 ){
              brightness = 0;
            } else{
              brightness = fadeMax;
            }
          }

          analogWrite(_pin, brightness);
        }
        break;
      }
      case  FadeLed::BLINK: {
        if (millis() - lastBlinkTime >= blinkInterval) {
          // save the last time you blinked the LED
          lastBlinkTime = millis();
          ledState = !ledState;
        }

        digitalWrite(_pin, ledState);

        break;
      }  
    }
  }
  
};


class Button {
  private:
    String _label;
    int _pin;
    unsigned long _delay;
    unsigned long _lastDebounceTime;
    int _lastStateBtn = HIGH;
    bool _setState = false; // The  button state tht is considered "set"
    bool _wasSet = false; // Flagged when button is True
    bool _wasUnset = false; // Flagged when button is True

    bool isTimeToUpdate(){
      return (millis() - _lastDebounceTime) > _delay;
    }

  public:
    Button(String label, int pin, bool setState = true, unsigned long delay=125) : 
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

    int checkStateChange(){

      if (checkSetAndClear()){
        return 1;
      } else if (checkUnsetAndClear()){
        return 0;
      } else {
        return -1;
      }
      
    }
    
    
    int stateChanged(){}
};



class StateMachine {

  private: 
    static const int OFF_STATE = 10;

    void (StateMachine::*current_state)() = &StateMachine::initState;

    int lowPowerPin; // Enable 48 Volt power supplies
    int highPowerPin; // Enable 80 Volt power supplies

    FadeLed led;
   
    Button buttons[NBUTTONS];

    String stateLabel;
    String lastStateLabel;

    #define SB_LEN 100
    char str_buf[SB_LEN];
    

    bool stateInit = false; // Has the new state been initialized?

    bool checkReset() { return buttons[0].state()==true; }
    bool checkKeyOn() { return buttons[1].state()==true; }
    bool checkKeyOff(){ return buttons[1].state()==false; }
    bool checkEStop() { return buttons[2].state()==false; }
    bool checkPower() { return buttons[3].state()==true; }

    bool checkAbort(){
      return checkKeyOff() | checkEStop();
    }

    void initState(){
      stateLabel = "Init";
      
    }
    
    void offState() {
      stateLabel = "off";

      if (!stateInit){
        turnOffHighPower();
        turnOffLowPower();
        led.setOff();
        stateInit=true;
        
      } else {
        
        if (checkKeyOn()){   
          toOnState();  
        }
      }
    }

    void onState() {
      stateLabel = "on";

      if (!stateInit){
        turnOffHighPower();
        turnOffLowPower();
        led.setFadeInterval(50);
        led.setFade(10);
        stateInit=true;
        
      } else {
        if (checkKeyOff()){
          toOffState();
        }  else if (checkReset()){
          toOnState();
        }  else if (checkPower()){
          toLowPowerOnState();  
        }
      }
    }
    
    void lowPowerOnState() {
      stateLabel = "lowpower";

      if (!stateInit){
        turnOffHighPower();
        turnOnLowPower();
        led.setBlink(1000);
        stateInit=true;
      } else {    
        if(checkKeyOff()){
          toOffState();  
        } else if (checkPower()){
          toFullPowerOnState();
        } else if (checkReset()){
          toOnState();
        } 

      }
      
    }
    void fullPowerOnState()  {
      stateLabel = "highpower";
      
      if (!stateInit){
        turnOnHighPower();
        turnOnLowPower();
        led.setOn();
        stateInit=true;
      } else {    
        if(checkKeyOff()){
          toOffState();  
        } else if ( checkPower()){
          toLowPowerOnState();
        } else if(checkEStop() ){
          toErrorState();
        }  else if (checkReset()){
          toOnState();
        } 
      }
      
    }
    
    void errorState(){
      stateLabel = "error";
      
      if (!stateInit){
        turnOffHighPower();
        
        led.setBlink(125);
        stateInit=true;
      } else {    
        if(checkKeyOff()){
          toOffState();  
        } else if ( checkPower()){
          toLowPowerOnState();
        }  else if (checkReset()){
          toOnState();
        } 
      }
      
    }
    
    void shuttingDownState(){
      stateLabel = "shutdown";
    }

    void setRunCurrentState(void (StateMachine::*new_current_state)()){
      current_state = new_current_state;
      
      stateInit = false;
      // Initialize the state
      execState();
      
     
    }

    void turnOnLowPower(){
      digitalWrite(lowPowerPin, HIGH);
    }
    void turnOnHighPower(){
      digitalWrite(highPowerPin, HIGH);
    }
    void turnOffLowPower(){
      digitalWrite(lowPowerPin, LOW);
    }
    void turnOffHighPower(){
      digitalWrite(highPowerPin, LOW);
    }

    
  public:

    StateMachine(int ledPin, int powerPin, int estopPin, int keyPin, int resetPin, 
                 int highPowerPin, int lowPowerPin) : 
      led(ledPin), 
      buttons( {
        Button("reset",resetPin, false),
        Button("key",keyPin, false),
        Button("estop",estopPin, false),
        Button("power",powerPin, false)}),
        
      highPowerPin(highPowerPin), lowPowerPin(lowPowerPin) {

      pinMode(highPowerPin, OUTPUT);
      pinMode(lowPowerPin, OUTPUT);

      updateButtons();

      lastStateLabel = "__init__";
     
    }

    // Transition to other states
    void toOnState() {
      setRunCurrentState(&StateMachine::onState);
    }
      
    void toOffState() {
      setRunCurrentState(&StateMachine::offState);
    }

    void toLowPowerOnState()  {
      setRunCurrentState(&StateMachine::lowPowerOnState);
    }
    void toFullPowerOnState()  {
      setRunCurrentState(&StateMachine::fullPowerOnState);
    }
    void toErrorState() {
      setRunCurrentState(&StateMachine::errorState);
    }
    void toShuttingDownState() {
      setRunCurrentState(&StateMachine::shuttingDownState);
    }

    void execState(){
      (((StateMachine*)this)->*StateMachine::current_state) ();
    }


    bool commandStateChange(String ns){

      if (ns == "off") toOffState();
      else if (ns == "on") toOnState();
      else if (ns == "lowpower") toLowPowerOnState();
      else if (ns == "highpower") toFullPowerOnState();
      else if (ns == "error") toErrorState();
      else return false;

      return true;
      
    }

    void buttonsChanged(){

        //printButtons();

        // Possibly transition states
        execState();
       
    }

    bool updateButtons(){
      bool stateChanged = false;      
      
      for(int i = 0; i < NBUTTONS; i++){
        buttons[i].update();
        if ( buttons[i].checkStateChange() != -1){
          stateChanged = true;
        }
      }

      return stateChanged;
    }

    bool update(){

      bool stateChanged = updateButtons();

      led.update();

      if(stateChanged){
        buttonsChanged();
      }

      return stateChanged;
    }

    void printButtons(){
        //pt("State: "); pt(stateLabel);pt(" Buttons: "); ps;
        for(int i = 0; i < NBUTTONS; i++){
          pt(buttons[i].label());pt(":");ps;pt(buttons[i].state());ps;
        }
       
    }


    char* getStateLabel(){
      stateLabel.toCharArray(str_buf, SB_LEN);
      return str_buf;
    }


    bool stateChanged(){
      if (stateLabel != lastStateLabel){
        lastStateLabel = stateLabel;
        return true;
      }
      return false;
    }
};

StateMachine sm(3,9, 10, 11, 12, 43, 45) ;

SoftwareSerial alt_serial(7, 6); // RX, TX

void setup() {
  Serial.begin(9600);
  alt_serial.begin(9600);
  
  alt_serial.println(">init");
  Serial.println(">init");
  
}


void loop() {

  String command;


  sm.update(); // Update leds for fading and blinking

  // Check front-side serial for commands. These will come from a human user. 
  if(Serial.available()) {
    command=Serial.readString();// read the incoming data as string
    command.trim();
  }

  // If the operator didn't command anything, check the back-side serial, 
  // which is commands from the controller. 
 
  if (command.length() == 0 && alt_serial.available()) {
      command=alt_serial.readString();// read the incoming data as string
      command.trim();
  }

  // If we got a command, display it, on both serial connections 
  if (command.length() > 0){
    if(sm.commandStateChange(command)){
        Serial.print(">"); Serial.println(command);
        alt_serial.print(">"); alt_serial.println(command);
    } else {
        // Bad command, or did not change state
        Serial.print("-"); Serial.println(command);
        alt_serial.print("-"); alt_serial.println(command);
    }
  }

  // Print changes in state to both serials. 
  if (sm.stateChanged()){
    alt_serial.print("<");alt_serial.println(sm.getStateLabel());
    Serial.print("<");Serial.println(sm.getStateLabel());
   
  }

}
