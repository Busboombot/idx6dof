#include <iostream>
#include <unity.h>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <time.h>


#include "trj_planner.h"
#include "trj_segment.h"

// Can't get platform.io to load this library
// #include "FastCRC.h"

using namespace std;

extern time_t time(time_t *);

// These are supposed to be defined somewhere, but aren't
unsigned __exidx_start;
unsigned __exidx_end;

void setUp(){
    
}
void tearDown(){
    
}

vector<Move> get2Moves(){
    return std::vector<Move>{
        Move( 0, {10000,100}),
        Move( 0, {10000,100})
    };
}

vector<Joint> getJoints(){
    return std::vector<Joint>{
        Joint(0, 10e3, 300e3),
        Joint(1, 10e3, 300e3)
     
    };
}

struct RunOut {
    int n = 0;
    uint32_t t = 0; // total_time
    uint32_t crc = 0;
};

void test_move_object(){

    Move move( 0, {10000,100});

    cout << "Move Object "<< move.x[0] << " " << sizeof(move) << endl;

}

// Test simple pushes to the queue. 
void test_push_to_deque(){

    Planner p = Planner(getJoints());

    for(const auto& move : get2Moves())
        p.push(move);
      
    TEST_ASSERT_EQUAL(2, p.getSegmentsSize());

}

void test_positive_time(){
 
    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );

    Move move(2);
   
    move.x[0] = 1000;
  
    move.x[1] = 1000;
 
    p.push(move);
  
    cout << p << " " << p.getQueueTime() << endl;

}

// Test a simple case with two moves. 
void test_basic_init(){

    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );

    p.push(Move( 0, {10000, 100}));
    p.push(Move( 0, {10000, 100}));

    cout << p <<  endl;

    SubSegments3 *ss = p.peekSegment(0).getSubSegments3();

    TEST_ASSERT_EQUAL(0, (*ss)[0].a.v_0 );
    TEST_ASSERT_EQUAL(167, (*ss)[0].a.x );
    TEST_ASSERT_EQUAL(10000, (*ss)[0].a.v_1 );

    TEST_ASSERT_EQUAL(10000, (*ss)[0].c.v_0 );
    TEST_ASSERT_EQUAL(9667, (*ss)[0].c.x );
    TEST_ASSERT_EQUAL(10000, (*ss)[0].c.v_1 );

    TEST_ASSERT_EQUAL(0, (*ss)[1].a.v_0 );
    TEST_ASSERT_EQUAL(2, (*ss)[1].a.x );
    TEST_ASSERT_EQUAL(100, (*ss)[1].a.v_1 );

    TEST_ASSERT_EQUAL(100, (*ss)[1].c.v_0 );
    TEST_ASSERT_EQUAL(97, (*ss)[1].c.x );
    TEST_ASSERT_EQUAL(100, (*ss)[1].c.v_1 );

    delete ss;

}



struct RunOut run_out(Planner &p, bool print = false){

    struct RunOut ro;
    //FastCRC32 CRC32;

    int n = 0;

    do {
        const PhaseJoints pj = p.getNextPhase();

        if(print)
            cout << pj << " ql="<< p.getQueueSize()<< " qt="<< p.getQueueTime() << endl;
        ro.n++;
        ro.t += pj.t;
        int crcv[3] = {pj.moves[0].x, pj.moves[0].v_0, pj.moves[0].t};
        // Stuipd fake CRC because I can't get platformio to link to 
        // the FastCRC library. 
       
        for(int i=0; i< sizeof(crcv)/sizeof(crcv[0]); i++ ){
            ro.crc += crcv[i];
        }
            
    } while (p.getQueueSize() > 0);

    return ro;
}

void test_phase_iter(){

    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );

    p.push(Move( 0, {10000, 100}));
    p.push(Move( 0, {10000, 100}));

    RunOut  ro = run_out(p, true);

    TEST_ASSERT_EQUAL(6, ro.n);
    TEST_ASSERT_EQUAL(2136202, ro.crc);
    TEST_ASSERT_EQUAL(0, p.getQueueTime());
    TEST_ASSERT_EQUAL(0, p.getQueueSize());

    p.push(Move( 0, {10000, 100}));
    p.push(Move( 0, {10000, 100}));

    cout << p <<  endl;

    // After clearingin everything out and adding new segments, the velocity
    // of the first block should be zero. 
    
    const PhaseJoints &pj = p.getNextPhase(); ;

    TEST_ASSERT_EQUAL(0, pj.moves[0].v_0);
    TEST_ASSERT_EQUAL(167, pj.moves[0].x);

    run_out(p);

    if(false){
        cout << "============" << endl;

        Planner p2 = Planner( {Joint(0, 20e3, 300e3), Joint(0, 20e3, 300e3)} );
        p2.push(Move( 0, {6000, 0}));
        p2.push(Move( 0, {0, 6000}));
        p2.push(Move( 0, {-6000, 0}));
        p2.push(Move( 0, {0, -6000}));
        p2.push(Move( 0, {6000, 0}));
        p2.push(Move( 0, {0, 6000}));
        p2.push(Move( 0, {-6000, 0}));
        p2.push(Move( 0, {0, -6000}));

        cout << p2 << endl;

        run_out(p2, true);
    }

}

// Put a lot of random moves into the planner and time how long it takes. 
void test_big_rand(){

    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );

    srand (time(NULL));

    auto start = chrono::steady_clock::now();

    int N = 1000;

    for(int i = 0; i < N; i++ ){
        p.push(Move( 0, {rand()%10000, rand()%10000}));
    }
    cout << "--> "  << p.getQueueTime() << " " << p.getQueueSize() << endl;
    
    const PhaseJoints *pj;

    do {
        p.getNextPhase(); 
    } while (p.getCurrentPhase().ssn!=SubSegName::NONE);

    cout << "<-- " << p.getQueueTime() << " " << p.getQueueSize() << endl;
    
    TEST_ASSERT_EQUAL(0, p.getQueueTime());
    TEST_ASSERT_EQUAL(0, p.getQueueSize());

    auto end = chrono::steady_clock::now();

    auto ms = chrono::duration <double, micro> ((end-start)).count();

    cout << "TIME: " << (ms/N)  << " ms per move " << endl;

}

void test_zero_move(){

    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );

    p.push(Move( 0, {10000, 100}));
    p.push(Move( 0, {0, 100}));
    p.push(Move( 0, {10000, 100}));

    cout << p <<  endl;

    int i = 0;
    for(; p.getNextPhase().ssn != SubSegName::NONE;i++);

    // SHould be 8, not 9, because one phase with no motion was removed. 
    TEST_ASSERT_EQUAL(7, i);
}

// Test that we can set the time per move, and the planner will basically
// respect it. In the first case, each move should execute in about 1s, 
// while in the second, they should execute as fast as possible. 
void test_time_limit(){
   
    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );
    
    p.push(Move(1e6, {1000, 1000}));
    p.push(Move(1e6, {2000, 2000}));
    p.push(Move(1e6, {5000, 5000}));

    TEST_ASSERT_TRUE(p.getQueueTime()> 3e6);

    run_out(p);

    p.push(Move(0, {1000, 1000}));
    p.push(Move(0, {2000, 2000}));
    p.push(Move(0, {5000, 5000}));

    cout << p <<  endl;
    TEST_ASSERT_TRUE(p.getQueueTime() < 9e5);

}

void test_negative(){

    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );

    p.push(Move(1e6, {1000, 1000}));
    p.push(Move(1e6, {-2000, -2000}));
    p.push(Move(1e6, {5000, 5000}));

    cout << p <<  endl;
    
}

Move jog(int x){
    return Move(0,1e5, Move::MoveType::jog, {x,x});
}

void test_jog_ql(){
    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );
    
    p.push(jog(1000));
    TEST_ASSERT_EQUAL(3, p.getQueueSize());
    TEST_ASSERT_EQUAL(1000, p.getPosition()[0]);
    
    p.push(jog(1000));
    TEST_ASSERT_EQUAL(6, p.getQueueSize());
    TEST_ASSERT_EQUAL(2000, p.getPosition()[0]);
    
    p.push(jog(1000));
    TEST_ASSERT_EQUAL(6, p.getQueueSize());
    TEST_ASSERT_EQUAL(2000, p.getPosition()[0]);
    
    p.push(jog(1000));
    TEST_ASSERT_EQUAL(6, p.getQueueSize());
    TEST_ASSERT_EQUAL(2000, p.getPosition()[0]);
  
}

void test_abs_rel(){
    Planner p = Planner( {Joint(0, 10e3, 300e3), Joint(0, 10e3, 300e3)} );
    
    int x = 1000;
    
    p.push(Move(0,1e5, Move::MoveType::relative, {x,x}));
    TEST_ASSERT_EQUAL(1000, p.getPosition()[0]);
    
    p.push(Move(0,1e5, Move::MoveType::relative, {x,x}));
    TEST_ASSERT_EQUAL(2000, p.getPosition()[0]);
    
    p.push(Move(0,1e5, Move::MoveType::absolute, {x,x}));
    TEST_ASSERT_EQUAL(1000, p.getPosition()[0]);
    
    p.push(Move(0,1e5, Move::MoveType::absolute, {x,x}));
    TEST_ASSERT_EQUAL(1000, p.getPosition()[0]);
    
    p.push(Move(0,1e5, Move::MoveType::relative, {x,x}));
    TEST_ASSERT_EQUAL(2000, p.getPosition()[0]);
    
}

int main(){

    UNITY_BEGIN();   

    RUN_TEST(test_move_object);
    RUN_TEST(test_push_to_deque);
    RUN_TEST(test_basic_init);
    RUN_TEST(test_phase_iter);
    RUN_TEST(test_big_rand);
    RUN_TEST(test_zero_move);
    RUN_TEST(test_positive_time);
    RUN_TEST(test_time_limit);
    RUN_TEST(test_negative);
    RUN_TEST(test_jog_ql);
    RUN_TEST(test_abs_rel);

    UNITY_END();
    return 0;
}


