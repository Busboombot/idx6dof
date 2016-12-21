
#ifndef AccelStepper_h
#define AccelStepper_h

#include <Arduino.h>

#define fastSet(pin) (digitalPinToPort(pin)->PIO_SODR |= digitalPinToBitMask(pin) ) 
#define fastClear(pin) (digitalPinToPort(pin)->PIO_CODR |= digitalPinToBitMask(pin) )

#include <stdlib.h>     /* abs */
#include <math.h>       /* sqrt and fabs  */

class IDXStepper
{
    
    
private:

    uint8_t  stepPin;
    uint8_t directionPin;

    int8_t direction; 
    
    int32_t stepsLeft;
    uint32_t lastTime;
   
    uint32_t position = 0;
   
    float t=0;
    int16_t v0;
    int16_t v1;

    float a;
    long n;
    float cn;
    
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

        t = 0;
        v0 =0;
        v1 = 0;
        stepsLeft = 0;
        
        a = 0;
        n = 0;
        cn = 0.0;
        
        pinMode(stepPin, OUTPUT);
        pinMode(directionPin, OUTPUT);
    }
    

    inline void setParams(uint32_t segment_time, int16_t v0, int16_t v1, long stepsLeft){
        
        this->v0 = v0;
        this->v1 = v1;
        this->stepsLeft = abs(stepsLeft);
        
        direction = (stepsLeft > 0) ? CW : ((stepsLeft < 0) ? CCW: 0);
        
        t = ((float)segment_time)/1000000.0;
        
        if (v0==0 && v1==0){
            a = 0;
            n = 0;
            cn = 0;
            this->stepsLeft = 0;
        } else if (v0==0) {
            a = fabs((float)v1) / t;
            n = 0; // n will always be positive, so accelerating
            cn = 0.676 * sqrt(2.0 / abs(a)) * 1000000.0; // c0 in Equation 15
        } else {
            a = fabs((float)v1-(float)v0) / t;
            n = abs((long) ((v0 * v0) / (2.0 * a))); // Equation 16
            cn = 1000000.0 / abs(v0);
            
            // Need to put the sign back on n; n must be negative for deceleration
            if (abs(v1) < abs(v0)){
                n = -n;
            }
        }
        
        if (direction == CW ){
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
            
            // cn is always positive, but n can be negative. n is always stepped +1, 
            // so a negative n causes cn to get larger each step ->  deceleration
            // a positive n causes cn to get smaller each step -> acceleration
            
            cn = abs(cn - ( (2.0 * cn) / ((4.0 * n) + 1))); // Why is it going negative?
            
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
