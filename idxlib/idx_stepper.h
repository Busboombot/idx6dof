
#ifndef AccelStepper_h
#define AccelStepper_h

#include <Arduino.h>

#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )


class IDXStepper
{
    
    
private:

    uint8_t  stepPin;
    uint8_t directionPin;

    byte direction; 
    uint32_t stepsLeft;
    uint32_t lastTime;
   
    uint32_t position = 0
   
    long n; // Step counter for interval calculations
    float cn; // Interval, in microseconds
    
public:

    typedef enum
    {
	    CCW = -1,  ///< Clockwise
        CW  = 1   ///< Counter-Clockwise
    } Direction;
    

    IDXStepper(uint8_t stepPin, uint8_t directionPin) 
        : stepPin(stepPin), directionPin(directionPin) {
    
        direction = CW;

        lastTime = 0;

        n = 0;
        cn = 0.0;
        
        pinMode(stepPin, OUTPUT);
        pinMode(directionPin, OUTPUT);

    }
    

    
    inline void setParams(long n, float cn, long stepsLeft){
        this->stepsLeft = stepsLeft;
        this->direction = (stepsLeft > 0) ? CW : ((stepsLeft < 0) ? CCW: 0);
        this->n = n;
        this->cn = cn;
    
        if (this->direction == CW ){
            fastSet(directionPin);
        } else {
            fastClear(directionPin);
        }
    
    }
    
    inline uint32_t getStepsLeft(){
        return stepsLeft;
    }
    
    inline uint32_t getPosition(){
        return position;
    }
    
    
    inline long step(uint32_t now){
        
        
        if ( stepsLeft != 0 && ( (unsigned long)(now - lastTime)   > cn) ) {
            
            cn = cn - ( (2.0 * cn) / ((4.0 * n) + 1));  
            
            stepsLeft -= 1;
            n += 1;

            fastSet(stepPin);
            lastTime = now;
            
            position += direction;

        }
        
        return stepsLeft;
    }
    
    //Set the step line to low
    inline void clearStep(){

        fastClear(stepPin);
        
    }
    

protected:






};

#endif 
