from __future__ import print_function
from .messages import *

import serial
import serial.threaded

import struct
from time import sleep
import traceback
import sys
from . import SegmentList


class ResponseReader(serial.threaded.Protocol):
    sync_str_n = struct.pack('<2c', *Command.sync_str)

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

        while (sync_idx >= 0 and len(self.buf) >= (sync_idx + Response.size)):

            response = Response(self.buf[sync_idx:sync_idx + Response.size])

            if response.code == Response.RESPONSE_ACK:
                try:
                    self.sent[int(response.seq)].state = Response.RESPONSE_ACK
                    # print ("ACK", response)
                except KeyError as e:
                    print("ERROR: Got ack, but no message for seq: {}. Sent list has: {}  "
                          .format(response.seq), self.sent.keys())

            elif response.code == Response.RESPONSE_DONE:
                try:
                    del self.sent[int(response.seq)]
                    # print ("DONE", response)

                    self.callback(self.proto, response)
                except KeyError:
                    print("ERROR: No message for seq: {}".format(response.seq))

            else:
                print("ERROR: Unknown message type: " + str(response.code))

            self.buf = self.buf[sync_idx + Response.size:]

            self.event.set()

    def sync_pos(self):
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
            # print ("Wrote ", msg)
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



class ThreadedProto(object):
    def __init__(self, port, n_axes=6, a_max=500000, v_max=15000, callback=None):
        self.port = port

        baud = 1050000
        self.ser = serial.Serial(self.port, baud, timeout=1);

        self.segment_list = SegmentList(n_axes, v_max=v_max, a_max=a_max, d_max=None)

        def null_callback(proto, resp):
            pass

        self.callback = callback if callback is not None else null_callback

    def open(self):
        def proto_factory():
            return ResponseReader(self, self.callback)

        self.rr = serial.threaded.ReaderThread(self.ser, proto_factory)
        self.proto = self.rr.__enter__()
        return self.proto


    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_val, exc_tb):
        return self.rr.__exit__(exc_type, exc_val, exc_tb)

    def close(self):
        self.ser.close()
        return self.rr.__exit__(None, None, None)

    def write(self, data):
        self.proto.write(data)

    def wait(self, timeout=None):
        self.rr.event.clear()
        self.rr.event.wait(timeout)
        return len(self.rr)
