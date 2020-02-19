#pragma once

#include <vector>
#include <iostream>
#include "trj_planner.h"
#include "trj_util.h"
#include "trj_jointss.h"

using namespace std;

/// Segment, working data for one move for all axes
/* Segment: One move for all joints, with Accel, Cruise and Decel phases. 
 * 
 *
 */

class Segment {

public: 

    int n;

    float target_t = 0; // time the seg should take to execute.

    float t = 0;
    float t_a = 0;
    float t_c = 0;
    float t_d = 0;

    Segment *next = 0;
    Segment *prior = 0;

    std::vector<JointSegment> joint_segments;

    std::vector<Joint> joints;

    bool sign_change = false;

    friend ostream &operator<<( ostream &output, const Segment &s );

public:

    Segment(std::vector<Joint> joints, Segment* prior, const Move& move);

    // Return segment time in ticks ( microseconds )
    int32_t getTicks(){
        //ser_printf("TICKS %f %d", t, SEC_TO_TICKS(t));
        //cout << " TICKS " << t << " " << SEC_TO_TICKS(t) << endl;
        return SEC_TO_TICKS(t);
    }

    void update_second_to_last();
    void update_last(Segment* prior);

    // Return all joint segments in a single structure, for debug printing. 
    SubSegments3 *getSubSegments3();

    // Fill a given phase joints record
    void getPhaseJoints(PhaseJoints& pj, int phase);

    MoveArray getMoves();

};