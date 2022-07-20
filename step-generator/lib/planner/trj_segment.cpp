
#include <functional>
#include <math.h> // abs
#include <algorithm>    // std::min

#include "trj_segment.h"
#include "trj_jointss.h"


Segment::Segment(std::vector<Joint> joints, Segment* prior,  const Move& move): 
        n(move.seq), target_t(move.t) {

    int axis_n = 0;
   
    for(Joint& joint : joints){
        joint_segments.emplace_back(axis_n, joint, this,move.x[axis_n]);
        axis_n++;
    }

    if (prior){
        
        // Iterate through the JointSegements for this semgnet, and the prior one:
        // 1) Link the two together through next and prior links
        // 2) Check if any of the axes have change direction, which will
        //    force the boundary velocity to zero. 
        jsIter prior_js = prior->joint_segments.begin();
    
        for(JointSegment& js : joint_segments){
            
            (*prior_js).next_js = &js;
            js.prior_js = &(*prior_js);
            
            sign_change = sign_change or !same_sign((*prior_js).sign, js.sign );

            prior_js++;
        }
        
        prior->update_second_to_last();
        
    }

    update_last(prior);
}


void Segment::update_last(Segment* prior){

    const JointSegment *longest = &joint_segments[0];

    float max_t_min = 0;
    for(JointSegment &js : joint_segments){
        js.update_t_min();

        if (js.t_min > max_t_min){
            max_t_min = js.t_min;
            longest = &js; 
        }
    }

    t_a = longest->t_a_min;
    t_c = longest->t_c_min;
    t_d = longest->t_d_min;
    
    t = longest->t_min;

    for(JointSegment &js : joint_segments){
        js.update_sub_segments();
        js.update_end_velocity_limit(true);
        js.update_start_velocity_limit(prior==0, sign_change);
    }
    
    if (prior != 0){
        int i=0;
        for(JointSegment& prior_js : prior->joint_segments){
            JointSegment &next_js =  joint_segments[i];
            next_js.update_boundary_velocity(&prior_js, NULL);
            prior_js.update_boundary_velocity(NULL, &next_js);
            i++;
        }
        
    }
    
    for(JointSegment &js : joint_segments){
        js.update_v_c();
    }
}

void Segment::update_second_to_last(){

        for(JointSegment &js : joint_segments){
            js.update_sub_segments();
            js.update_end_velocity_limit(false);
        }

        for(JointSegment &js : joint_segments){
            js.update_v_c();
        }
}

// Return a new Subsegments vector
SubSegments3 *Segment::getSubSegments3(){

    SubSegments3 *v = new SubSegments3();

    for(JointSegment& js : joint_segments)
        v->push_back(js.getSubSegments3());

    return v;
}

void Segment::getPhaseJoints(PhaseJoints& pj, int phase){
   
    pj.seq = n;

    switch (phase){

        case 0: 
            pj.t = SEC_TO_TICKS(t_a);
            pj.ssn = SubSegName::ACCEL;
            break;
        case 1: 
            pj.t = SEC_TO_TICKS(t_c);
            pj.ssn = SubSegName::CRUISE;
            break;
        case 2: 
            pj.t = SEC_TO_TICKS(t_d);
            pj.ssn = SubSegName::DECEL;
            break;
    }

    for(JointSegment& js : joint_segments){
        js.loadJointSubSeg(pj.moves[js.n], pj.ssn);
    }

}

MoveArray Segment::getMoves(){
    MoveArray m;
    m.resize(joint_segments.size());
   
    for(unsigned int i = 0; i < joint_segments.size(); i++){
        m[i] = joint_segments[i].x;
    }
       
     return m; 
}


ostream& operator<<( ostream &output, const Segment &s ) { 

    output <<  std::setprecision(3) << s.n << " ["  
            << setw(5) << s.t_a << " " 
            << setw(5) << s.t_c << " " 
            << setw(5) << s.t_d << "]";

    for(const JointSegment& js : s.joint_segments)
        output << js.getSubSegments3();

    return output;      
}
