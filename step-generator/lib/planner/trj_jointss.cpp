
#include <cstdint> 
#include <deque>
#include <vector>
#include <array>
#include <iostream>
#include <iomanip>
#include <math.h> // rint,  abs
#include <algorithm>    // std::min
#include <functional>

#include <assert.h>

#include "trj_jointss.h"
#include "trj_stepper.h"
#include "trj_util.h"
#include "trj_segment.h"

using namespace std;


JointSegment::JointSegment(int n, Joint &joint, Segment *segment, int x_ ): 
    n(n), x(static_cast<float>(abs(x_))), sign(sgn(x_)), joint(joint), segment(segment) {

    // v_max is goverened by the joint settings, unless we have a target time, 
   
    if(segment->target_t != 0){
        v_max = min(joint.v_max, (float)x/((float)segment->target_t/(float)TIMEBASE));
    } else {
        v_max = joint.v_max;
    }

    v_c = v_max;
}

void JointSegment::update_start_velocity_limit(bool is_first, bool sign_change){

    if (is_first or sign_change or x == 0 or x_a + x_d >= x) {
        v_0_max = 0;
    } else {
        // Limit to either max joint velocity, or the small-x case.
        // The second term, x/t_a, is for when x is small, and v_c will be driven down to 0,
        // so all movement is in the accel and decel phase. x/t_a is
        // reduced from x=v_mean*t_a and v_mean  = 1/2 * (v_0 + v_c) for v_c = 0, etc.
        v_0_max = std::min(v_max, x / segment->t_a);
    }
}

void JointSegment::update_end_velocity_limit(bool is_last){
    if (is_last or x == 0 or x_a + x_d >= x){   
        v_1_max = 0;
    } else {
        v_1_max = std::min(v_max, x / segment->t_d);
    }
}


// How to calc the velocity: mean or v_c?
#define V v_c

void JointSegment::update_boundary_velocity(JointSegment *prior_js, JointSegment* next_js){

    
    
    if (prior_js != NULL){
        float mean_bv = (prior_js->V + V) / 2.;
        v_0 =  std::min(mean_bv, v_0_max);
        v_0 =  std::min(v_0, prior_js->v_1_max);
    } else if (next_js != NULL) {
        float mean_bv = (V + next_js->V) / 2.;
        v_1 = std::min(mean_bv, v_1_max);
        v_1 = std::min(v_1, next_js->v_0_max);
    }
    
    
    
}

float binary_search(std::function<float(float)> f, float v_min, float v_guess, float v_max){

    float old_guess;

    for(int i=0; i < 200; i++){

        float x = f(v_guess);
       
        if (roundf(x) > 0){
            old_guess = v_guess;
            v_guess = (v_max + v_guess) / 2.;
            v_min = old_guess;

        } else if (roundf(x) < 0){
            old_guess = v_guess;
            v_guess = (v_min + v_guess) / 2.;
            v_max = old_guess;

        } else {
            return v_guess;
        }

        if (fabs(v_min-v_max) < 1)
            return v_guess;
    }

    return NAN;
}

float JointSegment::update_sub_segments(){
    x_a = (v_0 + v_c) * segment->t_a / 2. ; 
    x_c = segment->t_c * v_c + x_err;
    x_d = (v_1 + v_c) * segment->t_d / 2. ;
    
    assert(!isnan(x_a));
    assert(!isnan(x_c));
    assert(!isnan(x_d));
    
    return x_a + x_c + x_d;
}

float JointSegment::search_v_c(){

        auto f = [this](float v_c){   
            
            float x_t = (v_0 + v_c) * segment->t_a / 2. +
                        (v_1 + v_c) * segment->t_d / 2. +
                        segment->t_c * v_c;

            return x -  x_t;

        };
        
        float v_mean =  segment->t != 0 ? x/segment->t : 0;
    
        assert(!isnan(v_mean));
    
        return binary_search(f,0,v_mean,v_max );
}

float accel_t_for_x(float x, float v_0, float a_max){

    float a,b,c, term_1, term_2, root_1, root_2;

    a = .5 * a_max;
    b = v_0;
    c = -x;

    term_1 = -b;
    term_2 = sqrt((b*b) - 4 * a * c);

    root_1 = (term_1 - term_2) / (2 * a);
    root_2 = (term_1 + term_2) / (2 * a);

    return std::max(root_1, root_2);
}

void JointSegment::update_t_min(){

        // These are over estimates of the t_a and t_d,
        // the max that the would need to be in any case.
        
        
        // If a max_t is set, it will result in a max velocity for
        // the whole segment. 
        if (segment->target_t != 0){

        }

        // Time to accelerate to max speed from initial speed
        float t_a = (abs(v_c - v_0)) / joint.a_max;
        // Time to decel from max speed to final speed.
        float t_d = (abs(v_c - v_1)) / joint.d_max;
       
        // Distances for the accel and decel phases
        float x_a = int(round((v_0 + v_c) * t_a / 2.));
        float x_d = int(round((v_1 + v_c) * t_d / 2.));

        // Not enough distance to accel to max speed -> triangle profile
        if (x_a + x_d > x){
            x_a = x_d = x / 2.; // assumes a_max and d_max are equal!   
            t_a = accel_t_for_x(x_a, v_0, joint.a_max);
            t_d = accel_t_for_x(x_d, v_1, joint.d_max);
        }


        float x_c = x - x_a - x_d;
        float t_c = 0;

        if (v_c != 0){
            t_c = x_c / v_c;
        } 

        t_min =   t_a + t_c + t_d;
        t_a_min = t_a;
        t_c_min = t_c;
        t_d_min = t_d;
        
}

void JointSegment::update_v_c(){
    x_err = 0;
   
    if (x == 0){
        v_c = 0;
        update_sub_segments();
        return;
    }

    v_c = search_v_c();
   
    assert(v_c >= 0);
   
    x_err = x - calc_x(v_c);

    assert(!isnan(v_c));
    assert(!isnan(x_err));

    update_sub_segments();

    return;
}

float JointSegment::mean_v(){
    return segment->t != 0 ? x / segment->t : 0;
}

float JointSegment::calc_x(float v_c){

    float x_a = (v_0 + v_c) * segment->t_a / 2.;
    float x_d = (v_1 + v_c) * segment->t_d / 2.;
    float x_c = segment->t_c * v_c;
    float x =  x_a + x_c + x_d;
    
    return x;

}

// Float Cast
#define FC(v) (static_cast<int>(rint(v)))

JointSubSegment3 JointSegment::getSubSegments3() const {
    return {
        {SEC_TO_TICKS(segment->t_a),FC(sign*x_a),FC(v_0),FC(v_c), SubSegName::ACCEL},
        {SEC_TO_TICKS(segment->t_c),FC(sign*x_c),FC(v_c),FC(v_c), SubSegName::CRUISE},
        {SEC_TO_TICKS(segment->t_d),FC(sign*x_d),FC(v_c),FC(v_1), SubSegName::DECEL}
    };
}

void JointSegment::loadJointSubSeg(JointSubSegment& jss, SubSegName phase)  {

    switch (phase){

        case SubSegName::ACCEL: 
            jss = {SEC_TO_TICKS(segment->t_a),FC(sign*x_a),FC(v_0),FC(v_c),phase};
            return;
        case SubSegName::CRUISE: 
            jss = {SEC_TO_TICKS(segment->t_c),FC(sign*x_c),FC(v_c),FC(v_c), phase};
            return;
        case SubSegName::DECEL: 
            jss = {SEC_TO_TICKS(segment->t_d),FC(sign*x_d),FC(v_c),FC(v_1), phase};
            return;
        case SubSegName::NONE:
            ; 

    }
}

ostream &operator<<( ostream &output, const PhaseJoints &pj ) { 

    output << "(" << setw(8) << pj.t << ")";
    for(const JointSubSegment &jss : pj.moves ){
        if(jss.ssn != SubSegName::NONE){
            output <<  "[" << static_cast<int>(jss.ssn) << " "
                << left << setw(6) << jss.v_0 << '\t'
                <<  jss.x << '/' << jss.t << "us" << '\t'
                << right << setw(6) << jss.v_1
                << "]";
        }
        
    }

    return output;
}


ostream &operator<<( ostream &output, const JointSubSegment3 &j ) { 
    output <<  "[" 
    << left << setw(6) << j.a.v_0
    << right << setw(6) << j.a.x
    << " : "
    << left << setw(6) << j.c.x << "@"
    << right << setw(6) << j.c.v_0
    << " : "
    << left << setw(6) << j.d.x 
    << right << setw(6) << j.d.v_1
    << "]";    

    return output;      
}
