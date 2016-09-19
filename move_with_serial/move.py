
import serial
import struct
import binascii
import time

class SerialPacketError(Exception):
    pass

ser = serial.Serial('/dev/cu.usbmodemFD1441', 115200, timeout=2);

def getNextPacket(ser):
    
    start = '....'
    
    while True:
        
        c = ser.read()
        if c:
            start = start[1:] + c
            if start == 'IDXC':
                break

    props = struct.unpack('<HHiii',ser.read(16))
    return props
  
code = 10
length = 10
for seq in range(10000):
    
    msg = ['I', 'D', 'X', 'C', code, length, seq, 0 , 0 ]
    ser.write(struct.pack('<4cHHiii', *msg) )
    
    props =  getNextPacket(ser)
    print  msg, props
    
    
    
    

