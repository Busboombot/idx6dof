
#include <functional>
#include <math.h> // abs
#include <algorithm>    // std::min
#include <iostream>
#include <vector>
#include <assert.h>

#include "trj_planner.h"
#include "trj_segment.h"
#include "trj_jointss.h"

using namespace std;

int here_count = 0; // for the HERE macro, in trj_util

Planner::Planner(std::vector<Joint> joints_){
    
    joints.resize(joints_.size());
    current_position.resize(joints.size());
    
    current_position = {0};
    
    int i = 0;
    for(Joint &j: joints_){
        j.n = i++;
        setJoint(j);
    }
}

Planner::Planner(){
    joints.resize(N_AXES);
    current_position.resize(joints.size());
}

void Planner::setJoint(Joint &joint){
    if((int)joint.n < (int)joints.size()){
        joints[joint.n] = joint;
    }
}

void Planner::push(const Move& move){ // push to tail

    Move move_ = move;

    if(move_.move_type == Move::MoveType::jog){
        // For jogs, we remove the last item, if there are more than two, 
        // and replace it. 
        
        if (  getQueueSize() >= 6 ){
            Segment* last = segments.back();
            segments.pop_back();
            queue_size -= 3;
            
            MoveArray last_moves = last->getMoves();
            
            for(unsigned int i=0; i < joints.size(); i++){
                current_position[i] -= last_moves[i];
            }
        } 
        // Now it is just a regular relative move_. 
        move_.move_type = Move::MoveType::relative;
    }
    
    // Transform the move from realtive to absolute. The result is that
    // the new position is the value of the move x position
    if(move_.move_type == Move::MoveType::absolute){
        for(unsigned int i=0; i < joints.size(); i++){
            int32_t x = move_.x[i]  ;
            move_.x[i]  -= current_position[i]; // absolute to relative
            current_position[i] = x; // current position is old absolute position
        }
    } else {  // relative
        for(unsigned int i=0; i < joints.size(); i++){
            current_position[i] += move_.x[i];
        }
    }
    
    Segment* last = 0;
    
    if (segments.size() > 0)
        last = segments.back();
    
    Segment *segment = new Segment(joints, last, move_);
  
    segments.push_back(segment);
  
    queue_time += segment->getTicks();
    queue_size += 3;
}


void Planner::push(int seq, int t, MoveArray x){ // push to tail
    push(Move(seq, t, Move::MoveType::relative, x));

}


const PhaseJoints& Planner::getCurrentPhase(){   
    return phase_joints;
}

const PhaseJoints& Planner::getNextPhase(){    

    if (segments.size() == 0){
        return phase_joints; // Should have an ssn of NONE; 
    }

    segments.front()->getPhaseJoints(phase_joints, current_phase++);
    queue_size--;
    queue_time -= phase_joints.t;
    
    // Some of the cruise phases have zero time; these are triange segments
    // We sould just skip over these. 
    if (phase_joints.ssn == SubSegName::CRUISE  and phase_joints.t == 0){
        queue_size--;
        segments.front()->getPhaseJoints(phase_joints, current_phase++);
    }

    if (current_phase == 3){  // 
        delete segments.front();
        segments.pop_front();
        current_phase = 0;

        if (segments.size() == 0){
            phase_joints.ssn = SubSegName::NONE;
            phase_joints.t = 0;
            
            queue_size = 0;
            queue_time = 0;
        }
    }

    return phase_joints;
}

ostream &operator<<( ostream &output, const Planner &p ) { 

    for(const Segment *s : p.segments)
        output << *s << endl;

    return output;
}

