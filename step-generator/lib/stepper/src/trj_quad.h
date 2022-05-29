
#pragma once 

#include <Arduino.h>
#include <stdlib.h>     /* abs */
#include <math.h>       /* sqrt and fabs  */
#include <limits.h>     /* LONG_MAX */
#include <binary.h>
#include "trj_bithacks.h"
#include "trj_fastset.h"
#include "trj_debug.h"
#include "trj_stepper.h"


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

public:
    
    inline StepInterface(uint8_t axis_n, uint8_t stepPin, uint8_t directionPin, 
     uint8_t enablePin): axis_n(axis_n), stepPin(stepPin), directionPin(directionPin), enablePin(enablePin) {
        pinMode(stepPin, OUTPUT);
        pinMode(directionPin, OUTPUT);
        pinMode(enablePin, OUTPUT);
    }
    
    virtual ~StepInterface(){}
    
    virtual inline void writeStep(){
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

    virtual inline void enable(){
        digitalWriteFast(enablePin, LOW);  // Usually active low 
    }
    
    virtual inline void enable(Direction dir){
        if(dir != STOP){
            digitalWriteFast(enablePin, LOW);  // Usually active low 
        }
        setDirection(dir);
    }
    
    virtual inline void disable(){
        digitalWriteFast(enablePin, HIGH);// Active low
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


// A version of the StepInterface that outputs quadrature, useful
// for testing encoder readers. 
class QuadratureInterface : public StepInterface {

    protected:

        int8_t steps[4] = {
            B10, // 2, 
            B11, // 3, 
            B01, // 1,
            B00};  // 0

    public:
        
        QuadratureInterface(uint8_t axis_n, uint8_t stepPin, uint8_t directionPin, uint8_t enablePin):
            StepInterface( axis_n,  stepPin,  directionPin,  enablePin){}
        
         ~QuadratureInterface(){}
        
        inline void writeStep(){
            int32_t apos = abs(position);
            int8_t bits = steps[apos % 4];
          
            
            digitalWriteFast(stepPin, bits >> 1);// Active low  
            digitalWriteFast(enablePin, bits & 1);// Active low  
            
            if(position%51 == 0)
                ser_printf("%d=%d %d=%d", stepPin ,(bits >> 1), enablePin, ( bits & 1) );
        
            position += direction;
         
        };
        
        inline void enable(){}
        inline void enable(Direction dir){setDirection(dir);}
        inline void disable(){setDirection(STOP); }
        
};
