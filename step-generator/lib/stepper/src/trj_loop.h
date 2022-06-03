#pragma once
#include <Arduino.h>

#include <limits.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "trj_messageprocessor.h"
#include "trj_fastset.h"
#include "trj_stepper.h"
#include "trj_ringbuffer.h"
#include "trj_bithacks.h"
#include "trj_config.h"
#include "trj_planner.h"

#define ITR_DELAY 2

#define EEPROM_OFFSET 1

class Loop {

public:

    Loop(Stream& serial) :  sdp(serial, *this)  {

    }

    void setup();

    void loopOnce();

    // Copy in a new config
    void setConfig(Config* config, bool eeprom_write=true);

    void setAxisConfig(AxisConfig* config, bool eeprom_write=true);

    void printInfo();

    void stop();

    void start();

    void enable();

    void setDirection();

    void disable();

    inline Config& getConfig(){ return config;}

    inline StepInterface& getStepper(uint8_t n){ return *steppers[n]; }

    inline StepperState& getState(uint8_t n){ return state[n]; }

    inline int step(uint8_t n){   
        return state[n].step(getStepper(n));
    }

    inline void reset(){}

    void isr();

    void clearIsr();

    void processMove(const uint8_t* buffer_, size_t size);

    int getLastSegNum(){
        return last_seg_num;
    }

private:

    Config config;
    AxisConfig axes_config[N_AXES];

    MessageProcessor sdp;
    
    IntervalTimer setTimer;
    IntervalTimer clearTimer;

    bool running = false;
    bool enabled = false;
    bool inPhase = false;

    Planner planner;
 
    uint32_t now;
  
    StepperState state[N_AXES];

    StepInterface* steppers[N_AXES] = {0,0,0,0,0,0};
    
    int last_seg_num = 0;

    void loopTick();

    void runSerial();

    void feedSteppers();

    bool isEmpty(){ return planner.isEmpty(); }

    bool nextPhase();

    void finishedPhase();



};