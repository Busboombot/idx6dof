
#pragma once

#include <cstdint> 
#include <deque>
#include <vector>
#include <array>
#include <iostream>
#include <iomanip>
#include <math.h> // rint
#include <initializer_list> 

#include "trj_jointss.h"
#include "trj_stepper.h"

#include "trj_util.h"

using std::array;
using std::cout;
using std::endl;
using std::setw;
using std::left;
using std::right;
using std::ostream;


class Segment;
class Joint;
class PhaseJoints;


/// A Move vector, which describes distances for all axes
/**
 * The Move Vector describes the distances to move for all
 * axes, plus the maximum velocity for the whole vector. 
 * Note: The max velocity parameter is not currently used. 
*/

using MoveArray = std::vector<int32_t> ;

struct Move {

    enum class MoveType {
        relative, 
        absolute, 
        jog
    };

    uint32_t seq = 0; 

    MoveType move_type = MoveType::relative;

    // Total Vector Time, in microseconds.
    uint32_t t = 0; 

    // Distances
    MoveArray x;

    Move(int n_joints):seq(0), move_type(MoveType::relative), t(0), x(){
        x.resize(n_joints);
    }

    Move(int n_joints, uint32_t seq, uint32_t t, int v): seq(seq), t(t), x(){
        x.resize(n_joints);
    }

    Move(uint32_t seq, uint32_t t, MoveType move_type, MoveArray x): move_type(move_type), t(t), x(x){}

    Move(uint32_t seq, uint32_t t, MoveType move_type, std::initializer_list<int> il): 
        move_type(move_type), t(t), x(MoveArray(il.begin(), il.end())){}
    
    Move(uint32_t t, std::initializer_list<int> il): Move(0, t, MoveType::relative, il){}
    
};


/// Trajectory Planner. Turns a sequence of moves into moves plus velocities
/*
*/

class Planner {

protected:

    // Joint configuration
    std::vector<Joint> joints;

    // Running segment number
    int seg_number;

    std::deque<Segment*> segments;

    int current_phase = 0;

    PhaseJoints phase_joints;

    int32_t queue_size=0;
    int32_t queue_time=0;

    MoveArray current_position;

public:

    Planner(std::vector<Joint> joints);
    
    Planner();

    // Reset the joints
    void setNJoints(int n_joints){
        joints.resize(n_joints); 
    }
    void setJoint(Joint &joint);

    const std::vector<Joint> &getJoints(){ return joints;}

    const std::deque<Segment*> & getSegments() { return segments; }

    // Add a move, processing it into a Segment
    void push(const Move& move);

    //void push(MoveArray x);

    void push(int seq, int t, MoveArray x);


    Segment& peekSegment(int i){
        return *segments[i];
    }

    int getSegmentsSize(){
        return segments.size();
    }

    uint32_t getQueueTime(){
        return queue_time;
    }

    uint32_t getQueueSize(){
        return  queue_size;
    }

    MoveArray getPosition(){
        return current_position;
    }

    // Return ref to an internal PhaseJoints, loaded with the parameters for
    // the current phase of a give axis. 
    const PhaseJoints&  getNextPhase();

    const PhaseJoints& getCurrentPhase();

    bool isEmpty(){ return segments.size() == 0; }

    friend ostream &operator<<( ostream &output, const Planner &p );

    

};
