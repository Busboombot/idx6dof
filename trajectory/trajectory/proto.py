from __future__ import print_function
import serial
import serial.threaded
import struct
import binascii
import time
from time import sleep
import traceback
import sys
from . import SegmentList

class SerialPacketError(Exception):
    pass


#struct command {
#    byte sync[2] = {'I','D'}; // 2
#    uint16_t seq = 0; // Packet sequence // 2
#    uint16_t code = 0; // command code // 2
#    uint16_t pad = 0; // padding // 2
#    uint32_t segment_time = 0; // total segment time, in microseconds // 4
#    int16_t v0[6] = {0,0,0,0,0,0}; // Initial segment velocity, in steps per second // 12
#    int16_t v1[6] = {0,0,0,0,0,0}; // Final segment velocity // 12
#    int32_t steps[6] = {0,0,0,0,0,0}; // number of steps in segment // 24
#    uint32_t crc = 0; // Payload CRC // 4
#}; // 64

class Command(object):
    
    COMMAND_MIN_VALUE = 10
    COMMAND_APOSITION = 10
    COMMAND_RPOSITION = 11
    COMMAND_VELOCITY = 12
    COMMAND_ACCELERATION = 13
    COMMAND_POSITIONQUERY = 20

    
    sync_str = (b'I',b'D')
    msg_fmt = ('<2c'+ # Sync code "ID"
              'H'+ # seq
              'H'+ # code
              'H'+ # padding
              'I'+ # segment time
              '6h'+ # v0 initial velocity
              '6h'+ # v1 final velocity
              '6i'+ # steps
              'I') # CRC )
    msg_header =  msg_fmt[:6]
    size = struct.calcsize(msg_fmt)
    
    def __init__(self, 
                 seq, code, segment_time, 
                 v0, v1, steps,
                 crc=None, state = None):
                 
        self.code = code
        self.seq = seq
        self.pad = 0xBEEF
        self.segment_time = segment_time
        
        self.v0 = v0
        self.v1 = v1
        self.steps = steps
        
        assert len(self.v0 ) == 6
        assert len(self.v1 ) == 6
        assert len(self.steps ) == 6
        
        self.crc = crc
        
        self.state = state
        
        
    class PopFront(object):
        def __init__(self, p):
            self._p = p

        def __getitem__(self, n):
            self._p, r = self._p[n:], self._p[:n]
            return r


    @staticmethod
    def decode(data):
        
        p = self.PopFront(struct.unpack(Command.msg_fmt,data))
        
        _ = p[2]
        seq, code, pad, segment_time = p[4]
        v0 = p[6]
        v1 = p[6]
        steps = p[6]
        crc = p[1]
        
        return Command(seq, code, segment_time, v0, v1, steps, crc)
        
    def encode(self):
        
        assert len(self.v0 ) == 6
        assert len(self.v1 ) == 6
        assert len(self.steps ) == 6
        
        msg = (list(self.sync_str) + 
              [self.seq, self.code, self.pad, self.segment_time] +
              self.v0 + self.v1 + self.steps)
              
        try:
            crc = binascii.crc32(struct.pack(Command.msg_fmt[:-1], *msg))
        except:
            print("CRC Failed for :", msg)
            raise
    
        try:
            self.crc = s32tou(crc)
        except:
            print("s32tou Failed for :", crc)
            raise
    
        msg.append(self.crc) # Add checksum on
        
        return struct.pack(self.msg_fmt, *msg)
        
    def __repr__(self):
        return '<Rqst #{} {} t={} v0={} v1={} x={} crc={} ({})>'.format(self.seq, self.code, 
                                                       self.segment_time, self.v0, self.v1, self.steps, 
                                                       self.crc, self.state)
        
      
class Response(object):
        
    RESPONSE_NACK = 0 
    RESPONSE_ACK = 1 
    RESPONSE_DONE = 2 
    
    sync_str = 'ID'
    msg_fmt = (
                '<2c'+ # Sync code "IDXC"
                'H'+ # seq
                'H'+ # code
                '7H'+ # queue_size through max_loop_time
                '6h'+ # Axis step positions
                '6i'+ # Encoder diffs
                'I' ) # CRC
                
    msg_header =  msg_fmt[:6]
    size = struct.calcsize(msg_fmt)
    
    def __init__(self, data):
        
        self.seq = 0 # command code # 2
        self.code = 0 # Packet sequence #2
    
        self.queue_size = None # 2
        self.queue_min_seq  = None # 2
        self.min_char_read_time  = None # 2
        self.max_char_read_time  = None # 2
        self.min_loop_time  = None # 2
        self.max_loop_time  = None # 2
        self.padding  = None # 2
        # 20 
        self.steps = [None]*6  # 24
        self.encoder_diffs = [None]*6 # 12
    
        self.crc = 0; # Payload CRC // 4
        
        self.state = None
        
        self.decode(data)
        
    def decode(self, data):
        
        p = struct.unpack(Response.msg_fmt,data)[2:]

        (self.seq, 
        self.code, 
        self.queue_size, 
        self.queue_min_seq,
        self.min_char_read_time, 
        self.max_char_read_time ,
        self.min_loop_time, 
        self.max_loop_time,
        self.padding 
        ) = p[0:9]
        
        self.encoder_diffs = p[9:15]
        self.steps = p[15:21] 
        
        
    def __repr__(self):
        return '<Resp #{} {} q({},{}) c({},{}) l({},{}) {} {} {} ({})>'.format(
        self.seq, self.code, 
        self.queue_size, self.queue_min_seq,
        self.min_char_read_time, self.max_char_read_time ,
        self.min_loop_time, self.max_loop_time, 
        self.steps, self.encoder_diffs, self.crc, self.state)
    

class ResponseReader(serial.threaded.Protocol):
    
    sync_str_n = struct.pack('<2c',*Command.sync_str)
    
    def __init__(self, proto, callback):
        import threading
        self.proto = proto
        self.callback = callback
        
        self.event = threading.Event()
    
    def connection_made(self, transport):
        self.buf = bytearray()
        self.sent = {}
        self.transport = transport

    def data_received(self, data):
        
        self.buf.extend(data) 
        
        sync_idx = self.buf.find(self.sync_str_n)

        while( sync_idx >= 0 and len(self.buf) >= (sync_idx+Response.size)):
            
            response = Response(self.buf[sync_idx:sync_idx+Response.size])

            if response.code == Response.RESPONSE_ACK:
                try:
                    self.sent[int(response.seq)].state = Response.RESPONSE_ACK
                    #print ("ACK", response)
                except KeyError as e:
                    print ("ERROR: Got ack, but no message for seq: {}. Sent list has: {}  "
                            .format(response.seq), self.sent.keys())
                  
            elif response.code == Response.RESPONSE_DONE:
                try:
                    del self.sent[int(response.seq)]
                    #print ("DONE", response)
                    
                    self.callback(self.proto, response)
                except KeyError:
                    print ("ERROR: No message for seq: {}".format(response.seq))
                
            else:
                print ("ERROR: Unknown message type: "+str(response.code))

            
            self.buf = self.buf[sync_idx+Response.size:]
            
            self.event.set()

        
    def sync_pos (self):
        return self.buf.find(self.sync_str_n)

    def connection_lost(self, exc):

        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')
        
    def write(self, msg):
        
        try:
            self.transport.write(msg.encode())
            msg.state = 'sent'
            self.sent[int(msg.seq)] = msg
            #print ("Wrote ", msg)
        except struct.error:
            print(msg)
            raise
            
    def sleep(self):
        """Sleep for a bit to let the other thread clear out recieved messages """
        
        
        if len(self.sent) > 4:
            while len(self.sent) > 2:
                sleep(.05)
                
    def wait(self, timeout=None):
        self.event.clear()
        self.event.wait(timeout)
        return len(self.sent)
                
    def __len__(self):
        return len(self.sent)
   
def s32tou(v):
    """Convert a signed 32 bit in to unsigned """
    return struct.unpack('I',struct.pack('i', v))[0]
  

class Proto(object):
    
    def __init__(self, port, n_axes = 6, a_max=500000, v_max=15000, callback=None):

        self.port = port
        
        baud = 1050000
        self.ser = serial.Serial(self.port, baud, timeout=1);
        
        self.segment_list = SegmentList(n_axes, v_max=v_max, a_max=a_max, d_max = None)

        
        def null_callback(proto, resp):
            pass
            
        self.callback = callback if callback is not None else null_callback
        
    def open(self):
        def proto_factory():
            return ResponseReader(self, self.callback)
        
        self.rr = serial.threaded.ReaderThread(self.ser, proto_factory )
        self.proto =  self.rr.__enter__()
        return self.proto

        

        
    def open(self):
        def proto_factory():
            return ResponseReader(self, self.callback)
        
        self.rr = serial.threaded.ReaderThread(self.ser, proto_factory )
        self.proto =  self.rr.__enter__()
        return self.proto
    
    
    def close(self):
        return self.rr.__exit__(None, None, None)
        
    def __enter__(self):
        return self.open()
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        return self.rr.__exit__(exc_type, exc_val, exc_tb)
    
    
    
    def close(self):
        self.ser.close()
        
        
    def write(self, data):
        self.proto.write(data)
        
    def wait(self, timeout=None):
        self.rr.event.clear()
        self.rr.event.wait(timeout)
        return len(self.rr)
        
        


        
        
  
  