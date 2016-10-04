
import serial
import serial.threaded
import struct
import binascii
import time


class SerialPacketError(Exception):
    pass

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
    msg_fmt = '<4cHHi6H6HI'
    size = struct.calcsize(msg_fmt)
    
    
    def __init__(self, seq, code,  directions, ticks, steps, crc=None, state = None):
        self.code = code
        self.seq = seq
        self.directions = directions
        self.ticks = ticks
        self.steps = steps
        self.crc = crc
        self.state = state
      
        
    @staticmethod
    def decode(data):
        
        p = struct.unpack(Message.msg_fmt,data)
        
        code, seq, directions  = p[4:7]
        ticks = p[7:13]
        steps = p[13:19]
        
        assert(len(ticks) == 6)
        assert(len(steps) == 6)
        
        crc = p[19]
        
        return Message(seq, code, directions, ticks, steps, crc)
        
    def encode(self):
        
        msg = list(self.sync_str) + [self.code, self.seq, self.directions] + self.ticks + self.steps
    
        try:
            self.crc = s32tou(binascii.crc32(struct.pack(Message.msg_fmt[:-1], *msg)))
        except:
            print msg
            raise
    
        msg.append(self.crc) # Add checksum on
        
        return struct.pack(self.msg_fmt, *msg)
        
    def __repr__(self):
        return '<{} {} {} {} {} {} ({})>'.format(self.seq, self.code, self.directions, 
                                            self.ticks, self.steps, self.crc,self.state)
        
        
class ResponseReader(serial.threaded.Protocol):
    
    
    sync_str_n = struct.pack('<4c',*Message.sync_str)
    
    def connection_made(self, transport):
        self.buf = bytearray()
        self.sent = {}

        self.transport = transport

    def data_received(self, data):
        
        self.buf.extend(data) 
        
        sync_idx = self.buf.find(self.sync_str_n)

        while( sync_idx >= 0 and len(self.buf) >= (sync_idx+Message.size)):
            message = Message.decode(self.buf[sync_idx:sync_idx+Message.size])
            
            if message.code == Message.COMMAND_ACK:
                try:
                    self.sent[message.seq].state = Message.COMMAND_ACK
                 
                except KeyError:
                    print ("ERROR: No message for seq: {}".format(message.seq))
                    
            elif message.code == Message.COMMAND_DONE:
                try:
                    del self.sent[message.seq]
                except KeyError:
                    print ("ERROR: No message for seq: {}".format(message.seq))
                
            else:
                print ("ERROR: Unknown message type: "+str(message))
            
            self.buf = self.buf[sync_idx+Message.size:]
         
        
    def sync_pos (self):
        return self.buf.find(self.sync_str_n)

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
  
  
class Proto(object):
    
    def __init__(self, port):
        self.port = port
        baud = 1050000
        self.ser = serial.Serial(self.port, baud, timeout=1);
        

    def __enter__(self):
        self.rr = serial.threaded.ReaderThread(self.ser, ResponseReader )
        self.proto =  self.rr.__enter__()
        return self.proto
        
    def __exit__(self, exc_type, exc_val, exc_tb):
    
        return self.rr.__exit__(exc_type, exc_val, exc_tb)
        
    def write(self, data):
        self.proto.write(data)
        
        
        
        
    
    
  