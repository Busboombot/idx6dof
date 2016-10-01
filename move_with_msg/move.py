#! /usr/local/bin/python

import serial
import serial.threaded
import struct
import binascii
import time

class SerialPacketError(Exception):
    pass


baud = 1050000
ser = serial.Serial('/dev/cu.usbmodemFD1441', baud, timeout=1);

class Message(object):
    
    COMMAND_NACK = 0 
    COMMAND_ACK = 1 
    COMMAND_DONE = 2 
    COMMAND_APOSITION = 10
    COMMAND_RPOSITION = 11
    COMMAND_VELOCITY = 12
    COMMAND_ACCELERATION = 13
    COMMAND_POSITIONQUERY = 20
    
    sync_str = 'IDXC'
    msg_fmt = '<4cHHi6f6iI'
    size = struct.calcsize(msg_fmt)
    
    
    def __init__(self, seq, code,  directions, positions, velocities, crc=None, state = None):
        self.code = code
        self.seq = seq
        self.directions = directions
        self.positions = positions
        self.velocities = velocities
        self.crc = crc
        self.state = state
      
        
    @staticmethod
    def decode(data):
        
        p = struct.unpack(Message.msg_fmt,data)
        
        ( x, x, x, x, code, seq, directions)  = p[:7]
        
        p = p[7:]
        
        velocities = p[:6]
        p = p[6:]
        positions = p[:6]
        
        crc = p[6]
        
        return Message(seq, code, directions, positions, velocities, crc)
        
    def encode(self):
        
        msg = list(self.sync_str) + [self.code, self.seq, self.directions] + self.velocities + self.positions
    
        self.crc = s32tou(binascii.crc32(struct.pack(Message.msg_fmt[:-1], *msg)))
    
        msg.append(self.crc) # Add checksum on
        
        return struct.pack(self.msg_fmt, *msg)
        
    def __repr__(self):
        return '<{} {} {} {} {} {}>'.format(self.seq, self.code, self.directions, 
                                            self.positions, self.velocities, self.crc)
        
        

class ResponseReader(serial.threaded.Protocol):
    
    
    sync_str_n = struct.pack('<4c',*Message.sync_str)
    
    def connection_made(self, transport):
        self.buf = bytearray()
        self.sent = {}
        self.recieved = {}
        self.transport = transport

    def data_received(self, data):
        
        self.buf.extend(data) 
        
        sync_idx = self.buf.find(self.sync_str_n)
        
        if sync_idx >= 0 and len(self.buf) >= (sync_idx+Message.size):
            message = Message.decode(self.buf[sync_idx:sync_idx+Message.size])
            
            if message.code == Message.COMMAND_ACK:
                self.sent[message.seq].state = Message.COMMAND_ACK
            elif message.code == Message.COMMAND_DONE:
                del self.sent[message.seq]
            else:
                raise Exception("Unknown message type: "+str(message))
            
            self.buf = self.buf[sync_idx+Message.size:]
         
         

    def connection_lost(self, exc):
        import traceback
        import sys
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')
        
    def write(self, msg):
        
        try:
            self.transport.write(msg.encode())
            msg.state = 'sent'
            self.sent[msg.seq] = msg
            print ("Wrote ", msg)
        except struct.error:
            print(msg)
            raise
   
def s32tou(v):
    """Convert a signed 32 bit in to unsigned """
    return struct.unpack('I',struct.pack('i', v))[0]
  
code = 10
directions = 57
positions = [1,2,3,4,5,6]
velocities=[10,20,30,40,50,60]

with serial.threaded.ReaderThread(ser, ResponseReader ) as proto:

    for seq in range(10):

        msg = Message(seq, code,  directions, positions, velocities)
        
        proto.write(msg)
    
    print proto.sent
            
    time.sleep(2)
    
    print proto.sent

    
        
    
    
    
    

