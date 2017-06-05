from __future__ import print_function
from .messages import *

import serial
import serial.threaded

import struct
from time import sleep, time
import traceback
import sys
from . import SegmentList
from .util import s32tou
from Queue import Queue


class TimeoutException(Exception):
    pass

def timeout_handler(signum, frame):
    raise TimeoutException()


class Proto(object):

    sync_str_n = struct.pack('<2c', *Command.sync_str)

    def __init__(self, port, n_axes=6, a_max=50000, v_max=10000, callback=None, timeout=0):
        self.port = port

        baud = 1050000
        self.ser = serial.Serial(self.port, baud, timeout=timeout);

        self.segment_list = SegmentList(n_axes, v_max=v_max, a_max=a_max, d_max=None)

        def null_callback(proto, resp):
            pass

        self.callback = callback if callback is not None else null_callback

        self.buf = bytearray()
        self.sent = {}
        self.acks = []
        self.dones = []

        self.last_ack = 0
        self.last_done = 0

        self.queue_length = 0
        self.queue_time = 0

    def open(self):
        pass


    def close(self):
        self.ser.close()

    def purge(self):
        """Read all remaining data"""

        while self.ser.read():
            print ("Purging data")


    def __enter__(self):

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


    def write(self, msg):

        try:
            self.ser.write(msg.encode())
            msg.state = 'sent'
            self.sent[int(msg.seq)] = msg
            # print ("Wrote ", msg)
        except struct.error:
            print(msg)
            raise

        while True:
            
            if self.read_next() is False:
                break

            if len(self.acks) > 0:
                ack = self.acks.pop(0)
                
                break
 

    def wait_done(self, seq):
        """Wait until a sequence is done"""


        while True:

            if self.last_done >= seq:
                return True

            if self.read_next() is False:
                break

            try:
                done = self.dones.pop(0)
                

            except IndexError:
                pass


        return False

    def wait_backlog(self, n):
        """Read next until the backlog is less than n"""


        while True:
            if self.read_next() is False:
                break
                
            if len(self.sent) <= n:
                return

    def read_next(self, timeout=None):
        """Read a response, ACK or DONE. Returns when input is exhausted or a response is available. """

        while True:

            d = self.ser.read()

            if not d:
                return False

            self.buf.extend(d)

            sync_idx = self.buf.find(self.sync_str_n)

            if (sync_idx >= 0 and len(self.buf) >= (sync_idx + Response.size)):

                if sync_idx != 0:
                    # The sync string isn't the first two chars in the buffer, so there is garbage.
                    print("GARB", self.buf[:sync_idx])

                response = Response(self.buf[sync_idx:sync_idx + Response.size])

                if response.code == Response.RESPONSE_ACK:
                    try:
                        self.sent[int(response.seq)].state = Response.RESPONSE_ACK
                        self.acks.append(response)
                        self.callback(self, response)
                        self.queue_length = response.queue_size
                        self.queue_time = response.queue_time
                        self.last_ack = response.seq
                        #print("ACK ", response)
                    except KeyError as e:
                        print("ERROR: Got ack, but no message for seq: {}. Sent list has: {}  "
                              .format(response.seq), self.sent.keys())

                elif response.code == Response.RESPONSE_DONE:
                    try:
                        del self.sent[int(response.seq)]
                        self.dones.append(response)
                    
                        self.callback(self, response)
                    
                        self.queue_length = response.queue_size
                        self.queue_time = response.queue_time
                        self.last_done = response.seq
                        #print("DONE ", response)
                    
                    except KeyError:
                        print("ERROR: Got DONE for unknown seq: {}".format(response.seq))

                else:
                    print("ERROR: Unknown message type: " + str(response.code))

                self.buf = self.buf[sync_idx + Response.size:]

                return response




