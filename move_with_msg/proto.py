
import serial
import serial.threaded
import struct
import binascii
import time


class SerialPacketError(Exception):
    pass


class Command(object):
    
    COMMAND_MIN_VALUE = 10
    COMMAND_APOSITION = 10
    COMMAND_RPOSITION = 11
    COMMAND_VELOCITY = 12
    COMMAND_ACCELERATION = 13
    COMMAND_POSITIONQUERY = 20
    
    sync_str = 'IDXC'
    msg_fmt = ('<4c'+ # Sync code "IDXC"
              'H'+ # code
              'H'+ # seq
              'I'+ # segment_time
              '6l'+ # n
              '6l'+ # steps left
              '6f'+  # cn
              'I'+ # CRC )
    msg_header =  msg_fmt[:6]
    size = struct.calcsize(msg_fmt)
    
    
    def __init__(self, seq, code,  segment_time, accelerations, velocities, steps, crc=None, state = None):
        self.code = code
        self.seq = seq
        self.segment_time = segment_time
        self.velocities = velocities
        self.steps = steps
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
        
        _ = p[4]
        code, seq , segment_time = p[3]
        accelerations = p[6]
        velocities = p[6]
        steps = p[6]
        crc = p[1]
        
        return Command(seq, code, accelerations, velocities, steps, crc)
        
    def encode(self):
        
        msg = list(self.sync_str) + [self.code, self.seq, self.segment_time] + self.ticks + self.steps
    
        try:
            self.crc = s32tou(binascii.crc32(struct.pack(Command.msg_fmt[:-1], *msg)))
        except:
            print msg
            raise
    
        msg.append(self.crc) # Add checksum on
        
        return struct.pack(self.msg_fmt, *msg)
        
    def __repr__(self):
        return '<{} {} {} {} {} {} ({})>'.format(self.seq, self.code, self.directions, 
                                            self.ticks, self.steps, self.crc,self.state)
        
      
class Response(object):
        
    RESPONSE_NACK = 0 
    RESPONSE_ACK = 1 
    RESPONSE_DONE = 2 
    
    sync_str = 'IDXC'
    msg_fmt = ('<4c'+
                'H'+
                'H'+
                '6H'+
                '6i'+
                '6h'+
                'I' )
    msg_header =  msg_fmt[:6]
    size = struct.calcsize(msg_fmt)
    
    def __init__(self, data):
        
        self.code = 0 # command code # 2
        self.seq = 0 # Packet sequence #2
    
        self.queue_size = None # 2
        self.queue_min_seq  = None # 2
        self.min_char_read_time  = None # 2
        self.max_char_read_time  = None # 2
        self.min_loop_time  = None # 2
        self.max_loop_time  = None # 2
        # 20 
        self.steps = [None]*6  # 24
        self.encoder_diffs = [None]*6 # 12
    
        self.crc = 0; # Payload CRC // 4
        
        self.state = None
        
        self.decode(data)
        
    def decode(self, data):
        
        p = struct.unpack(Response.msg_fmt,data)[4:]

        
        (self.code, self.seq, 
        self.queue_size, self.queue_min_seq,
        self.min_char_read_time, self.max_char_read_time ,
        self.min_loop_time, self.max_loop_time 
        ) = p[0:8]
        
        self.steps = p[8:14]
        self.encoder_diffs = p[14:20]
        
    def __repr__(self):
        return '<#{} {} q({},{}) c({},{}) l({},{}) {} {} {} ({})>'.format(
        self.seq, self.code, 
        self.queue_size, self.queue_min_seq,
        self.min_char_read_time, self.max_char_read_time ,
        self.min_loop_time, self.max_loop_time, 
        self.steps, self.encoder_diffs, self.crc, self.state)
    

class ResponseReader(serial.threaded.Protocol):
    
    
    sync_str_n = struct.pack('<4c',*Command.sync_str)
    
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
                    self.sent[response.seq].state = Response.RESPONSE_ACK
                    #print ("ACK", response)
                except KeyError:
                    print ("ERROR: No message for seq: {}".format(response.seq))
                    
            elif response.code == Response.RESPONSE_DONE:
                try:
                    del self.sent[response.seq]
                    print ("DONE", response)
                except KeyError:
                    print ("ERROR: No message for seq: {}".format(response.seq))
                
            else:
                print ("ERROR: Unknown message type: "+str(response.code))
                print sync_idx, len(self.buf)
                print (response)
            
            self.buf = self.buf[sync_idx+Response.size:]
         
        
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
            #print ("Wrote ", msg)
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
        
    @staticmethod
    def seg_dist_time(v0, x,t):
        """Return parameters for a segment given the initial velocity, distance traveled, and transit time."""
   
        v0 = float(v0)
        x = float(x)
        t = float(t)
        
        a = 2.*(x-v0*t)/(t*t)

        v1 = v0 + a*t

        return abs(x), v0, v1, t, a

    @staticmethod
    def seg_velocity_time(v0,v1,t):
        """ Return parameters for a segment given the initial velocity, final velocity, and transit time. """
        
        v0 = float(v0)
        v1 = float(v1)
        x = float(t)
        
        a = (v1-v0)/t

        x = a * (t**2) / 2.

        return abs(x), v0, v1, t, a

    @staticmethod
    def seg_velocity_dist(v0, v1, x):
        """Return segment parameters given the initial velocity, final velocity, and distance traveled. """

        v0 = float(v0)
        v1 = float(v1)
        x = float(x)

        if v0 != v1:
            t = abs(2.*x / (v1+v0))
            a = (v1-v0)/t 
        else:
            t = abs(x/v0)
            a = 0

        return abs(x), v0, v1, t, a
        
        
        
        
    
    
  