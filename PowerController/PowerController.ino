#include "Arduino.h"

/*
  Based on:
  Button.h - Library for Button Debounce.
  Created by Maykon L. Capellari, September 30, 2017.
  Released into the public domain.
*/


#include <stdio.h>
#include <string.h>
#include <SoftwareSerial.h>  

#define pt(x) Serial.print(x)
#define pl(x) Serial.println(x)
#define ps Serial.print(' ')
#define pnl Serial.print('\n')

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
    bool _setState = false; // The  button state that is considered "set"
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
    
    
    int stateChanged(){} // Override to do someting when button changes
};


#define NBUTTONS 6
#define RESET_BUTTON_IDX 0 // Front panel Reset/Error button
#define KEY_BUTTON_IDX 1
#define ESTOP1_BUTTON_IDX 2
#define ESTOP2_BUTTON_IDX 3
#define ESTOPEN_BUTTON_IDX 4
#define POWER_BUTTON_IDX 5  // Blue power button. 

class StateMachine {

  private: 
    static const int OFF_STATE = 10;

    void (StateMachine::*current_state)() = &StateMachine::initState;

    int lowPowerPin; // Enable 48 Volt power supplies
    int highPowerPin; // Enable 80 Volt power supplies

    FadeLed led;
    FadeLed boardLed;
   
    Button buttons[NBUTTONS];

    String stateLabel;
    String lastStateLabel;

    #define SB_LEN 100
    char str_buf[SB_LEN];
    
    bool stateInit = false; // Has the new state been initialized?

    // Unless the controller is connected to USB power, the entire controller turns on
    // and off wth the key switch. 
    bool checkKeyOn() { return buttons[KEY_BUTTON_IDX].state()==true; }
    bool checkKeyOff(){ return buttons[KEY_BUTTON_IDX].state()==false; }
    bool checkReset() { return buttons[RESET_BUTTON_IDX].state()==true; }
    bool checkEStop() { return buttons[ESTOP1_BUTTON_IDX].state()==false &&
                               buttons[ESTOP2_BUTTON_IDX].state()==false &&
                               buttons[ESTOPEN_BUTTON_IDX].state()!=true; }
    bool checkEStopEn() { return buttons[ESTOPEN_BUTTON_IDX].state()==true; }
    bool checkPower() { return buttons[POWER_BUTTON_IDX].state()==true; }


    void initState(){
      stateLabel = String("Init");
      
    }

    void offState() {
      stateLabel = String("on");

      if (!stateInit){
        turnOffHighPower();
        turnOffLowPower();
        led.setOff();
        boardLed.setOff();
        stateInit=true;
        
      } else {
        if (checkKeyOn()){
          toOnState();
        } 
      }
    }
    
    void onState() {
      stateLabel = String("on");

      if (!stateInit){
        turnOffHighPower();
        turnOffLowPower();
        led.setFadeInterval(50);
        led.setFade(10);
        boardLed.setOn();
        stateInit=true;
        
      } else {
        if (checkKeyOff()){
          toOffState();
        } else if (checkReset()){
          toOnState();
        }  else if (checkPower()){
          toLowPowerOnState();  
        }
      }
    }
    
    void lowPowerOnState() {
      stateLabel = String("lowpower");

      if (!stateInit){
        turnOffHighPower();
        turnOnLowPower();
        led.setBlink(1000);
        stateInit=true;
      } else {    
        if (checkKeyOff()){
          toOffState();
        } else if(checkEStop() ){
          toErrorState();
        } else if (checkPower()){
          toFullPowerOnState();
        } else if (checkReset()){
          toOnState();
        } 

      }
      
    }
    void fullPowerOnState()  {
      stateLabel = String("highpower");
      
      if (!stateInit){
        turnOnHighPower();
        turnOnLowPower();
        led.setOn();
        stateInit=true;
      } else {    
        if (checkKeyOff()){
          toOffState();
        } else if(checkEStop() ){
          toErrorState();
        }  else if ( checkPower()){
          toLowPowerOnState();
        } else if (checkReset()){
          toOnState();
        } 
      }
      
    }
    
    void errorState(){
      stateLabel = String("error");
      
      if (!stateInit){
        turnOffHighPower();
        turnOffLowPower();
        led.setBlink(125);
        stateInit=true;
      } else {    
        if (checkKeyOff()){
          toOffState();
        } else if (checkReset()){
          toOnState();
        } 
      }
      
    }
    
    void shuttingDownState(){
      stateLabel = String("shutdown");
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


    StateMachine(int ledPin, int powerPin, int estopEnablePin, int estopPin1, int estopPin2, int keyPin, int resetPin, 
                 int highPowerPin, int lowPowerPin) : 
                 
      led(ledPin),
      boardLed(13), 
      
      buttons( {
        // The indexes of these button must match those
        // of the _BUTTON_INDEX defines
        Button(F("reset"),resetPin, false),
        Button(F("key"),keyPin, true),
        Button(F("estop1"),estopPin1, false),
        Button(F("estop2"),estopPin2, false),
        Button(F("estopEnable"),estopEnablePin, false),
        Button(F("power"),powerPin, false)}),
        
      highPowerPin(highPowerPin), lowPowerPin(lowPowerPin) {

      pinMode(highPowerPin, OUTPUT);
      pinMode(lowPowerPin, OUTPUT);

      updateButtons();

      lastStateLabel = F("__init__");
     
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


    // Change the state with a command from the serial port
    // These changes skip the state machine rules. 
    bool commandStateChange(String ns){

      if (ns == F("on")) toOnState();
      else if (ns == F("lowpower")) toLowPowerOnState();
      else if (ns == F("highpower")) toFullPowerOnState();
      else if (ns == F("error")) toErrorState();
      else if (ns == F("reset")) toOnState();
      
      else if (ns == F("?")) ; // Just query the state
      else return false;

      return true;
      
    }

    void buttonsChanged(){

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
       
        for(int i = 0; i < NBUTTONS; i++){
          pt(buttons[i].label());pt(":");ps;pt(buttons[i].state());ps;
        }
        pnl;
       
    }


    const String& getStateLabel(){
      return stateLabel; //.toCharArray(str_buf, SB_LEN);
      //return str_buf;
    }


    bool stateChanged(){
      if (stateLabel != lastStateLabel){
        lastStateLabel = stateLabel;
        return true;
      }
      return false;
    }
};


StateMachine sm(
  3, // LED  int ledPin, 
  8, // Power Switch int powerPin, 
  A1, // EStop Enable int estopEnablePin, 
  41, // EStop 1 (Conroller) int estopPin1, 
   9, // EStop 2 (Power) int estopPin1, 
  54, // Keyed On/Off Switch int keyPin, ( Not used?
  12, // Reset Switchint resetPin, 
  43, // High Power enable  int highPowerPin, 
  45  // Low Power Enable int lowPowerPin
  ) ;

  

#define COMMAND_BUF_LENGTH 40
String command; // char command[COMMAND_BUF_LENGTH];
String commandBuffers[] = {String(), String()};

void printCommand(const char *prefix, String str){
  Serial.print(prefix); Serial.println(str);
  Serial2.print(prefix); Serial2.println(str);
}

void setup() {
  
  sm.toOnState();

  command.reserve(COMMAND_BUF_LENGTH);
  commandBuffers[0].reserve(COMMAND_BUF_LENGTH);
  commandBuffers[1].reserve(COMMAND_BUF_LENGTH);
  
  Serial.begin(9600);
  Serial2.begin(9600);
  
  printCommand("<","init");
}

// Read a command from a serial port and stuff it into 
// the command string when done. 
void buildCommand(HardwareSerial& serial, String& str){
  while (serial.available() && command.length() == 0) { // Don't allow command to finish if a command is still active
    // get the new byte:
    char c = (char)serial.read();
   
    // add it to the inputString:

    if (c == '\n' || c == '\r' ||str.length() == COMMAND_BUF_LENGTH) {
      command = str;
      command.trim();
      str = "";
    } else {
      str += c;
      
    }
  }
}

void serialEvent() {
  buildCommand(Serial, commandBuffers[0]);
}

void serialEvent2() {
  //buildCommand(Serial2, commandBuffers[1]);
}

void loop() {

  
  // If we got a command, display it, on both serial connections 
  if (command.length() > 0){
    if(sm.commandStateChange(command)){ // True if the command is accepted
      printCommand(">", command);
    } else {
      // Bad command
      printCommand("-", command);
    }

    // Print changes in state to both serials. 
    if (command == "?"){
      printCommand("<", sm.getStateLabel());
      sm.printButtons();
      
    }

    command = ""; // Clear the command 
  }

  sm.update(); // Update leds for fading and blinking

  // Print changes in state to both serials. 
  if (sm.stateChanged()){
      printCommand("<", sm.getStateLabel());
  }



}
