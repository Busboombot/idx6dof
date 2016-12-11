// Structures for movement commands

#ifndef idx_command_h
#define idx_command_h

#include <Arduino.h>
#include <limits.h>
#include <LinkedList.h>  // https://github.com/ivanseidel/LinkedList
#include <CRC32.h>

#define IDX_COMMAND_NACK 0  // Failed to read payload
#define IDX_COMMAND_ACK 1   // Payload successfully stored in message list
#define IDX_COMMAND_DONE 2  // Command completed
#define IDX_COMMAND_APOSITION 10
#define IDX_COMMAND_RPOSITION 11
#define IDX_COMMAND_VELOCITY 12
#define IDX_COMMAND_ACCELERATION 13
#define IDX_COMMAND_POSITIONQUERY 20

#define N_AXES 6

// There is a practical minimum of 25 us per step == 40Ks/s
// If segment time is max of .065s, then max steps per segment is
// 2,600. With 24 bits, max segment time is 16sec, but is sensibly 
// capped at .82s, or 2**15s. Either way, a segment steps can be signed 16 bits. 

struct command {
    byte sync[2] = {'I','D'}; // 2
    uint16_t seq = 0; // Packet sequence // 2
    uint16_t code = 0; // command code // 2
    uint16_t segment_time = 0; // total segment time, in microseconds // 2

    uint16_t steps = {0,0,0,0,0,0}; // number of steps // 12
    int16_t accel_step = {0,0,0,0,0,0}; // "n" in the austin algorithm // 12
    int16_t intervals = {0,0,0,0,0,0}; // "Cn" in the Austin algorithm // 12
    
    uint32_t crc = 0; // Payload CRC // 4
}; // 48

struct axis_command {
    uint16_t seq = 0; // Packet sequence, segment number // 2
    uint8_t  axis = 0; // 1 high bit signals last axis in segment
    uint8_t  code = 0; // command code // 1
    uint16_t segment_time = 0; // total segment time, in microseconds // 2

    uint16_t steps = 0; // number of steps // 2
    int16_t accel_step = 0; // "n" in the austin algorithm // 2
    int16_t intervals = 0; // "Cn" in the Austin algorithm // 2
    
    uint32_t crc = 0; // Payload CRC // 4
}; // 16


struct response {
    byte sync[4] = {'I','D','X','C'};  // 4 
    uint16_t code = 0; // command code // 2
    uint16_t seq = 0; // Packet sequence //2
    
    uint16_t queue_size; // 2
    uint16_t queue_min_seq; // 2
    uint16_t min_char_read_time; // 2
    uint16_t max_char_read_time; // 2
    uint16_t min_loop_time; // 2
    uint16_t max_loop_time; // 2
    // 20 
    int32_t steps[N_AXES] = {0,0,0,0,0,0};  // 24
    int16_t encoder_diffs[N_AXES] = {0,0,0,0,0,0}; // 12
    
    uint32_t crc = 0; // Payload CRC // 4
}; // 60


/*
Total 64b
uint16 code
uint16 seq
union { 
    uint32[56] commands;
    struct {
        int32_t pad; //4
        int32_t directions =0 ; // Direction bits // 4
        float velocities[N_AXES] = {NAN,NAN,NAN,NAN,NAN,NAN}; // 24
        int32_t positions[N_AXES] = {0,0,0,0,0,0};  // 24
    }
}
uint32 crc
*/

uint16_t fletcher16( uint8_t const *data, size_t bytes );

class IDXCommandBuffer {

private:

    LinkedList<struct command*> commands;
    
    struct command *last_command;
    
    int buf_pos = 0;
   
    Serial_ &ser; // SerialUSB on the Arduino Due
    
    struct response cmd_response = {}; 
    
    uint32_t loop_start;
    uint32_t char_start;
    
    void send(struct command & command);
    
public:
    
    IDXCommandBuffer(Serial_ &ser) : ser(ser) {
        last_command =  new command();
        
    }

    // Wait for a header sync string, then read the entire header. 
    int run();

    inline int size(){
        return commands.size();
    }
    
    inline int buflen(){
        return buf_pos;
    }
    
    inline struct command * getMessage(){
        if ( size() == 0){
            return 0 ;
        }
        
        return commands.shift();
        
    }
    
    inline void startLoop(){
        loop_start = micros();
    }
    
    inline void endLoop(){
        
        uint32_t dt = micros()-loop_start;
        cmd_response.min_loop_time = (uint16_t)min(cmd_response.min_loop_time, dt);
        cmd_response.max_loop_time = (uint16_t)max(cmd_response.max_loop_time, dt);
    }
    
    inline void resetLoopTimes(){
        cmd_response.min_loop_time = USHRT_MAX;
        cmd_response.max_loop_time = 0;
    }
    
    inline void sendResponse(struct response & response, int seq, int code){ 
        
        response.seq = seq;
        response.code = code;  
        
        uint32_t crc  = CRC32::checksum( (const uint8_t*)&response, 
                                     sizeof(response) - sizeof(response.crc));
          
        ser.write( (uint8_t*)&response, sizeof(struct response));

    }

    inline void startCharRead(){
        char_start = micros();
    }
    
    inline void endCharRead(){
        
        uint32_t dt = micros()-char_start;
        cmd_response.min_char_read_time = (uint16_t)min(cmd_response.min_char_read_time, dt);
        cmd_response.max_char_read_time = (uint16_t)max(cmd_response.max_char_read_time, dt);
    }
    
    inline uint16_t queue_min_seq(){
        if (size() > 0){
            return (uint16_t)((struct command*)commands.get(0))->seq;
        } else {
            return 0;
        }
    }
    
    inline void resetCharReadTimes(){
        cmd_response.min_char_read_time = USHRT_MAX;
        cmd_response.max_char_read_time = 0;
    }

    inline void sendAck(struct command & command){ 
        cmd_response.queue_size = (uint16_t)size();
        cmd_response.queue_min_seq = queue_min_seq();
        sendResponse(cmd_response, command.seq, IDX_COMMAND_ACK);
    }

    inline void sendNack(struct command & command){ 
        cmd_response.queue_size = (uint16_t)size();
        cmd_response.queue_min_seq = queue_min_seq();
        sendResponse(cmd_response, command.seq, IDX_COMMAND_ACK);
    }

    inline void sendDone(struct command & command){ 
        cmd_response.queue_size = (uint16_t)size();
        cmd_response.queue_min_seq = queue_min_seq();
        sendResponse(cmd_response, command.seq, IDX_COMMAND_DONE);
    }
    
    inline void setPositions(int32_t (&positions)[N_AXES]){
        for (int i = 0; i < N_AXES; i++){
            cmd_response.steps[i] = positions[i];
        }
    }
    
};


class IDXCommandPort {
    
    public:
        
        inline IDXCommandPort(Serial_ &ser) : ser(ser) {
            
        }


        
        void recieveHeader(struct command_header & header){
            
        }
        

        
    private:
        
        Serial_ &ser;
        
    
};
    
#endif