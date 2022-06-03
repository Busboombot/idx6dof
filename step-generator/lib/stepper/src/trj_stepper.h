#pragma once 

#include <Arduino.h>
#include <stdlib.h>     /* abs */
#include <math.h>       /* sqrt and fabs  */
#include <limits.h>     /* LONG_MAX */
#include <binary.h>
#include "trj_bithacks.h"
#include "trj_fastset.h"
#include "trj_debug.h"

#define N_AXES 6 // Max number of axes

typedef enum
{
    CCW = -1,  ///< Clockwise
    STOP = 0,  ///< Clockwise
    CW  = 1   ///< Counter-Clockwise
} Direction;




// 2,000 rpm for a 1.8deg stepper is 400,000 steps per min, 7K steps 
// per sec. For a 10 ustep driver, 70KHz step pulse. 

#define TIMEBASE 1000000.0 // Microseconds

class StepInterface {
    
protected:
    
    uint8_t axis_n;
    bool pinState = true;
    
    uint8_t stepPin;
    uint8_t directionPin;
    uint8_t enablePin;

    int8_t direction = 0;
    int32_t position = 0;

    friend class StepperState;
    friend class Loop;

    uint8_t enable_active = 0;
    uint8_t enable_inactive = 1;

public:
    
    inline StepInterface(uint8_t axis_n, uint8_t stepPin, uint8_t directionPin, 
     uint8_t enablePin): axis_n(axis_n), stepPin(stepPin), directionPin(directionPin), enablePin(enablePin) {
        pinMode(stepPin, OUTPUT);
        pinMode(directionPin, OUTPUT);
        pinMode(enablePin, OUTPUT); // RESET signal doesn't work
    }
 
    
    inline void writeStep(){
        digitalWriteFast(stepPin, HIGH);
        position += direction;
    }
   
    inline void clearStep(){
        digitalWriteFast(stepPin, LOW);
    }
    
    inline void toggle(){
        if( (pinState = !pinState)){
            digitalWriteFast(stepPin, HIGH);
            position += direction;
        } else {
            digitalWriteFast(stepPin, LOW);
        }
    }

    inline void enable(){
       
        digitalWriteFast(enablePin, enable_active);  // Usually active low 
    }
    
    inline void enable(Direction dir){
        
        setDirection(dir);
        enable();
    }
    
    inline void disable(){
        digitalWriteFast(enablePin, enable_inactive);// Active low
        setDirection(STOP);
    }
    
    inline void setDirection(Direction dir){
        if (dir == CW){
            fastSet(directionPin);
            direction = CW;
        } else if (dir == CCW){
            fastClear(directionPin);
            direction = CCW;
        } else {
            direction = STOP;
        }
    }
    
    inline int getPosition(){ return position; }
    inline void setPosition(int64_t v){ position = v; }
     
};




class StepperState {
    
protected: 

    Direction direction=STOP; 
    uint32_t  stepsLeft=0;

    float delay_counter;
    float delay;
    float delay_inc;
    float v;
    float a;
    float t;   // Running time
    float t_s; // segment time, in sections
    float v_i; // Initial velocity

public:

    StepperState() {}
    
    StepperState(uint32_t segment_time, uint32_t v0, uint32_t v1, int32_t x, int32_t period) {
        setParams(segment_time, v0, v1, x, period);
    }
    
    inline uint32_t getStepsLeft(){
        return stepsLeft;
    }
    
    inline int getVelocity(){
        return v;
    }
    
    inline Direction getDirection() { return direction; }
    
    inline void setParams(uint32_t segment_time, uint32_t v0, uint32_t v1, int32_t x, int32_t period){

        if (x > 0){
            direction = CW;
        } else if (x < 0){
            direction = CCW;
        } else {
            direction = STOP;
        }
        t = 0; // cumulative time
        delay = 0;
        delay_counter = 0;
    
        if(v1 + v0 != 0 and abs(x) != 0){
            stepsLeft = abs(x);
            // This is the Dave Austin algorithm, via the AccelStepper arduino library. 
            t_s =  fabs(2.0 * ((float)x)) / ( ((float)v1) + ((float)v0 ) );
            v_i = (float)v0;
            a = ((float)v1-(float)v0) / t_s;
        } else {
            stepsLeft = 0;
            t_s = 0;
            v_i = 0;
            a = 0;
        }
        
        delay_inc = ((float)period)/((float)TIMEBASE);
        
    }
   

    inline bool step(StepInterface& stepper){


        if (stepsLeft == 0){
            return 0;
        }

        if (delay_counter >= delay){

            delay_counter -= delay;
            stepsLeft--;
            stepper.writeStep();
        }

        v = a * t + v_i;

        if(v != 0){
            delay = 1.0/v;
        } else {
            delay = 0;
        }
        delay_counter += delay_inc;
        t += delay_inc;

        return stepsLeft;
    }
        
};

class SubSegment {
public:
    uint16_t seq = 0;
    uint16_t code = 0;
    uint32_t segment_time = 0; // total segment time, in microseconds 
    uint32_t lastTime;
    StepperState axes[N_AXES];
public:
    SubSegment() {

    }
};

