
// Read a movement command from the serial port

#include "idx_command.h"
#include <Arduino.h>
#include "CRC32.h"

// Wait for a header sync string, then read the entire header. 
int IDXCommandBuffer::run(){
    
    if(ser.available()){
        startCharRead();
        
        char c =  ser.read();
        
        if (buf_pos < sizeof(last_command->sync)) {
           
            // When reading the sync string, ensure that each additional char in 
            // the buffer equals the one in the same position in the sync str. 
            // If not, the characters read so far aren't sync chars, so start over. 
            if ( c == last_command->sync[buf_pos] ){
                buf_pos++;
                
            } else{
                buf_pos = 0;
            }
       
        } else {
            
            *(char*)(((char*)last_command)+buf_pos) = c;
            
            buf_pos++;
            
            if (buf_pos == sizeof(struct command)){
                
                uint32_t crc  = CRC32::checksum( (const uint8_t*)last_command, 
                sizeof(*last_command) - sizeof(last_command->crc));
                 
                if (crc == last_command->crc){
                    commands.add(last_command);
                    sendAck(*last_command);
                    resetCharReadTimes();
                    //Serial.print("ACK ");Serial.println(last_command->seq);
                    last_command = new command();
                    
                } else {
                    sendNack(*last_command);
                }
           
                buf_pos = 0;
            }
        }
        endCharRead();
    }
    
    return buf_pos;
}



// From Wikipedia: https://en.wikipedia.org/wiki/Fletcher%27s_checksum
uint16_t fletcher16( uint8_t const *data, size_t bytes )
{
    uint16_t sum1 = 0xff, sum2 = 0xff;
    size_t tlen;
    
    while (bytes) {
        tlen = bytes >= 20 ? 20 : bytes;
        bytes -= tlen;
        do {
            sum2 += sum1 += *data++;
        } while (--tlen);
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
    }
    /* Second reduction step to reduce sums to 8 bits */
    sum1 = (sum1 & 0xff) + (sum1 >> 8);
    sum2 = (sum2 & 0xff) + (sum2 >> 8);
    return sum2 << 8 | sum1;
}

/*
# For python, a nasty hack based on 
# Wikipedia: https://en.wikipedia.org/wiki/Fletcher%27s_checksum
def fletcher16( data ):

    sum1 = 0xff
    sum2 = 0xff
    tlen = 0
    i = 0
    bytes = len(data)
    
    while (bytes):
        tlen = 20 if bytes >= 20 else bytes
        bytes -= tlen
        while True:
            sum1 += ord(data[i])
            sum2 += sum1
            i += 1
            tlen -= 1
            if tlen == 0:
                break
     
        sum1 = (sum1 & 0xff) + (sum1 >> 8)
        sum2 = (sum2 & 0xff) + (sum2 >> 8)
    
 
    sum1 = (sum1 & 0xff) + (sum1 >> 8)
    sum2 = (sum2 & 0xff) + (sum2 >> 8)
    return sum2 << 8 | sum1;
*/
