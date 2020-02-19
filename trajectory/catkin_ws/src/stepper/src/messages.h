#pragma once

#include <vector>
#include "serial/serial.h"

using std::vector;

#define N_AXES 6

enum class CommandCode : uint8_t {
  
  NONE =    0,
  ACK =     1,
  NACK =    2,
  RUN =     3,
  STOP =    4,
  RESET =   5,  // Payload is a debug message; the next packet is text
  MESSAGE = 6,  // Payload is a message; the next packet is text
  ERROR =   7,  // Some error
  ECHO  =   8,  // Echo the incomming header
  NOOP  =   9,  // Does nothing, but get ACKED
  CONFIG =  10, // Reset the configuration
  AXES   =  11,
  INFO   =  12,
  EMPTY  =  13, // Queue is empty, nothing to do. 

  DONE =    20,  // A Movemenbt comment is finished
  RMOVE =   21,  // A relative movement segment, with just the relative distance.   
  AMOVE =   22,  // An absolute movement
  JMOVE =   23   // A Jog movement. 

};


struct PacketHeader {
  uint16_t seq;       // Packet sequence number
  CommandCode code;   // Command code      
  uint8_t crc;    // Payload CRC8
  
  //PacketHeader(uint16_t seq, CommandCode code, uint8_t crc)
  
}; 


// Payload of the move command
struct Moves {
  uint32_t segment_time = 0; // total segment time, in microseconds // 4
  int32_t x[N_AXES];
}; // 8


struct CurrentState {
  int32_t queue_length = 0;
  uint32_t queue_time = 0;
  int32_t positions[N_AXES] = {0};
  int32_t planner_positions[N_AXES] = {0};
}; 


// Main Configuration class, 68 bytes
struct Config {

    uint8_t n_axes;         // Number of axes
    uint8_t interrupt_delay;    // How often interrupt is called, in microseconds
    bool debug_print;
    bool debug_tick;

    Config():
      n_axes(0), interrupt_delay(5), debug_print(false), debug_tick(false){}

    Config(uint8_t n_axes,uint8_t interrupt_delay,bool debug_print,bool debug_tick):
      n_axes(n_axes), interrupt_delay(interrupt_delay), debug_print(debug_print), debug_tick(debug_tick){}

};

struct AxisConfig {
    uint8_t axis;           // Axis number
    uint8_t step_pin;       // Step output, or quadture b
    uint8_t direction_pin;  // Direction output, or quadrature b
    uint8_t enable_pin;
    uint32_t v_max;
    uint32_t a_max;
};


#define MESSAGE_BUF_SIZE 256

class MessageProcessor;

class MessagePubHelper {

protected:
    
    MessageProcessor *mp;
    
public:
    
    void setMessageProcessor(MessageProcessor* mp){ this->mp=mp;}
    
    virtual void publish(PacketHeader &h, CurrentState &current_state) = 0;
    
    virtual void publish(PacketHeader &h) = 0;
    

};


class MessageProcessor {
    
protected:
    
    uint8_t data_buf[MESSAGE_BUF_SIZE]; // For composing messages
    uint8_t send_buf[MESSAGE_BUF_SIZE]; // Encoded and send
    uint8_t rcv_buf[MESSAGE_BUF_SIZE];  // encoded and recieved
    
    uint16_t seq = 0; // outgoing seq
    
    CommandCode last_code = CommandCode::NONE;
    int last_ack = 0;
    int last_done = 0;
    int last_seq = 0;
    
    
    CurrentState current_state;
    
    serial::Serial &ser;

    MessagePubHelper *mph=0;
    
    void handle(uint8_t* data, size_t len);
    
public: 
    
    MessageProcessor(serial::Serial &ser): ser(ser) {}

    void setMPH(MessagePubHelper *mph){ 
        this->mph = mph;
        mph->setMessageProcessor(this);
        
    }
    
    size_t send(CommandCode code, const uint8_t* payload, size_t payload_len);


    void sendConfig(const Config& config, std::vector<AxisConfig> axis_config);
    void sendConfig(const Config& config);
    void sendAxisConfig(const AxisConfig &axis_config);

    void sendInfo();

    void sendMove(CommandCode code, uint32_t t, vector<int> x);
    void sendMove(CommandCode code, uint32_t t, vector<double> x);
    void sendMove(CommandCode code, vector<int> x);
    void sendMove(CommandCode code, uint32_t t, std::initializer_list<int> il);
    void sendMove(CommandCode code, uint32_t t, std::initializer_list<double> il);

    void  aMove(vector<int> x);
    void  rMove(vector<int> x);
    void  jog(uint32_t t, vector<int> x);

    bool update();
    bool read_next(float timeout = .1);
    bool read_while_qt(float qt, float timeout);
    bool read_while_ql(int ql, float timeout);

    bool waitReadable(){ return ser.waitReadable(); }

    int getLastAck() { return last_ack;}
    int getLastDone() { return last_done;}
    int getLastSeq() {return last_seq;}
    CommandCode getLastCode() { return last_code;}
    int32_t getQueueLength(){return current_state.queue_length;}
    float getQueueTime(){return (float)current_state.queue_time/ (float)1e6;}

    std::vector<int32_t> getPositions(){ 
      return std::vector<int32_t>(current_state.positions, current_state.positions+N_AXES);
    }
   
    std::vector<int32_t> getPlannerPositions(){ 
      return std::vector<int32_t>(current_state.planner_positions, current_state.planner_positions+N_AXES);
    }
};





