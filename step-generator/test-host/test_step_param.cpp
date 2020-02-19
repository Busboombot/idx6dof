#include <iostream>
#include <iomanip> 
#include <stdio.h>
#include "trj_stepper.h"
#include "trj_bithacks.h"
#include <assert.h>

using std::cout;
using std::endl;
using std::setw;

void test_shift(int orig_v, uint8_t fpbits){
    int32_t v =  (orig_v  << fpbits);
    
    cout <<  FP_TOO_SMALL(v) << " " << FP_TOO_BIG(v) << endl;
    cout << "Before v="<< setw(14) << v <<  "\tfb="<< (int)fpbits << "\tv_shift=" << (v>>fpbits) << "\tv_orig=" <<  orig_v <<endl;
    adjust(fpbits, v);
    cout << "After v ="<< setw(14) << v <<  "\tfb="<< (int)fpbits << "\tv_shift=" << (v>>fpbits) << "\tv_orig=" <<  orig_v <<endl<<endl;
    
    assert( (v>>fpbits) == orig_v);
    
}

int main(){
    
    int t[] = {0x0001, 0x0010, 0x00FF,  1<<8,  0x0400, 0x0FFF, 0x7000, 0x7100, 0x7FFF ,  0x8000, 0xFFFF, 153485312, 0x60000000};
    
    //for(const int &i : t)
    //    std::cout << std::hex << std::setw(4) << i << " " << FP_TOO_SMALL(i) << " " << FP_TOO_BIG(i)  << std::dec << std::endl;

    //test_shift(1, 8); 
    //test_shift(4, 6);
    //test_shift(2342, 16);
    //test_shift(32767, 16);
    
    StepInterface interface(0,26,2, 30);
    
    float dt = 1;
    int v0 = 2000;
    int v1 = 0;
    int x  = (int) (((float)v0 + (float)v1) * dt)/2.0;
    
    StepperState state(dt*TIMEBASE, v0, v1, x); 
    //StepperState state(200000, 40000, 60000, 10000); 
    
    uint8_t mask = 0;
    int time = 0;
    int intr_period = 1;
    
    cout << "Initial n="<<state.n<<" "<<state.delay<< endl;
    
    while(state.getStepsLeft()){
        
        state.step(intr_period, 1, mask);
        
        if (time == 0 or mask){ //}&& state.getStepsLeft()%1000 == 0){
            std::cout
                << setw(10) << time 
                << " " << setw(7) << state.getStepsLeft() 
                << " lt=" << setw(10) << state.lastTime
                << " n=" << setw(6) << state.n 
                <<"\tdelay=" << state.delay 
                <<"\tca=" << state.ca
                <<"\tca1=" << setw(12) <<  state.ca1
                <<"\tfpb=" << (int)state.fp_bits
                << "\t -> " << (int)mask 
                <<"\tv=" << state.getVelocity()
                << std::endl;
        }
    
        time += intr_period;
        
        mask=0;
    }
    
}
