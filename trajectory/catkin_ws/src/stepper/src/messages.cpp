
#include "CRC.h"
#include "messages.h"
#include "cobs.h" 

#include <chrono>
#include <iostream>
#include <cstdio>
#include <algorithm>    // std::copy

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;



CRC::Table<std::uint8_t, 8> table(CRC::CRC_8());


size_t MessageProcessor::send(CommandCode code, const uint8_t* payload, size_t payload_len){
  
    PacketHeader h  = { seq,  code, 0};

    memcpy(data_buf, &h, sizeof(PacketHeader));
    size_t  data_len = sizeof(PacketHeader);
        
    if (payload_len > 0){
        memcpy(data_buf+sizeof(PacketHeader), payload, payload_len);
        data_len +=payload_len;
    }

    ((PacketHeader* )data_buf)->crc = CRC::Calculate(data_buf,data_len, table);

    size_t l = cobs_encode(data_buf, data_len, send_buf);
    send_buf[l] = 0;
    l++;

    size_t bytes_written = ser.write(send_buf, l);
    
    seq++;
   
    return bytes_written;
}

void MessageProcessor::sendInfo(){
    send(CommandCode::INFO, 0, 0);
    while(read_next(.2));
}

void MessageProcessor::sendConfig(const Config &config, std::vector<AxisConfig> axis_config){
    
    sendConfig(config);

    for(const AxisConfig &as : axis_config){
        sendAxisConfig(as);
    }

    while(read_next(.2));
}

void MessageProcessor::sendConfig(const Config &config){
    send(CommandCode::CONFIG, (const uint8_t*)&config, sizeof(config));
    while(read_next(.2));
}

void MessageProcessor::sendAxisConfig(const AxisConfig &axis_config){
    send(CommandCode::AXES, (const uint8_t*)&axis_config, sizeof(axis_config));
    while(read_next(.2));
}


void  MessageProcessor::sendMove(CommandCode code, uint32_t t, vector<int> x){
    Moves m;
    m.segment_time = t;
    
    int i = 0;
    for( auto xi : x)
        if(i < N_AXES)
            m.x[i++] = xi;   
    
    send(code, (const uint8_t*)&m, sizeof(m));

    current_state.queue_length += 3;
}

void  MessageProcessor::sendMove(CommandCode code, uint32_t t, vector<double> x){
    vector<int> vi;
    std::copy(x.begin(), x.end(), vi.begin());
    sendMove(code, t, vi);
}

void  MessageProcessor::sendMove(CommandCode code, vector<int> x) {
    sendMove(code, 0,x);

}
void  MessageProcessor::sendMove(CommandCode code, uint32_t t, std::initializer_list<int> il){
    sendMove(code, t, vector<int>(il.begin(), il.end()));
}
void  MessageProcessor::sendMove(CommandCode code, uint32_t t, std::initializer_list<double> il){
    vector<int> vi;
    std::copy(il.begin(), il.end(), vi.begin());
    sendMove(code, t, vi);
}

void  MessageProcessor::aMove(vector<int> x){ sendMove(CommandCode::AMOVE, 0, x); }
void  MessageProcessor::rMove(vector<int> x){ sendMove(CommandCode::RMOVE, 0, x); }
void  MessageProcessor::jog(uint32_t t, vector<int> x){ sendMove(CommandCode::JMOVE, t, x); }


bool MessageProcessor::update(){
    
    static int index = 0;
    
    while(ser.available()){
        ser.read(&rcv_buf[index], 1);
        if(rcv_buf[index] == 0){
            handle(rcv_buf, index);
            index = 0;
            return true;
        } else {
            index++;
        }
    }
    return false;
}

bool MessageProcessor::read_next(float timeout){

    auto t1 = std::chrono::steady_clock::now();
    
    while(true){
        if(update())
            return true;
        
        auto ts = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t1).count();
        
        if(ts > timeout){
            return false;
        }  
    }
}

// Read loop until the queue time is below a given value. 
bool MessageProcessor::read_while_qt(float qt, float timeout = 5){
    
    if(!read_next(timeout))
        return false;

    if (getQueueTime() < qt)
        return false;

    return true;
}

// Read while the queue size is larger than given value 
bool MessageProcessor::read_while_ql(int ql, float timeout = 2){
    
    if(!read_next(timeout))
        return false;

    if (getQueueLength() < ql)
        return false;

    return true;
}

void MessageProcessor::handle(uint8_t* data, size_t len){
    
    size_t bytes_decoded = cobs_decode(data, len, (uint8_t*)data_buf);
    
    PacketHeader &h = *((PacketHeader*)data_buf);
    
    // SHould check CRC here. 

    char * payload = ((char*)(data_buf+sizeof(PacketHeader)));
    payload[bytes_decoded-sizeof(PacketHeader)] = '\0'; // In case the payload is a string. 
    
    last_code = h.code;
    last_seq = h.seq;
    
    switch(h.code){
        
        case CommandCode::ACK: 
            last_ack = h.seq;
            memcpy((void*)&current_state, payload, sizeof(CurrentState));
            if(mph)
                mph->publish(h, current_state);
            break;
        
        case CommandCode::DONE: 
            last_done = h.seq;
        case CommandCode::EMPTY: 
            memcpy((void*)&current_state, payload, sizeof(CurrentState));
            if(mph)
                mph->publish(h, current_state);

       
        case CommandCode::NACK: 
            break;
        
        case CommandCode::ERROR: 
        case CommandCode::MESSAGE: 
            cout << payload << endl;
            break;
        
        case CommandCode::ECHO:
            cout << "ECHO "<< payload << endl; 
            break;
        
    }
    
}
