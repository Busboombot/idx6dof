from __future__ import print_function

import struct
from typing import List

from cobs import cobs

from trajectory.crc8 import crc8

TIMEBASE=1e6

class ProtoError(Exception):
    pass

class SerialPacketError(ProtoError):
    pass


class CRCError(ProtoError):
    pass

class BadMoveCodeError(ProtoError):
    pass

class CommandHeader(object):
    # struct Header {
    #   uint16_t seq; // Packet sequence number
    #   enum Command code : 6; // Command number
    #   enum AckNack ack_nack: 2;
    #   enum ReqResp req_resp: 1;
    #   enum FlowControl flow_control: 2;
    #   uint8_t n_axes: 5; // Number of axes
    #   uint32_t crc = 0; // Payload CRC // 4
    # }; // 8 bytes

    msg_fmt = ('<H' +  # seq
               'B' +  # Code
               'B')  # CRC8

    size = struct.calcsize(msg_fmt)

    # enum CommandCode  {
    CC_ACK      = 1
    CC_NACK     = 2
    CC_RUN      = 3
    CC_STOP     = 4
    CC_DEBUG    = 5     # Payload is a debug message; the next packet is text
    CC_MESSAGE  = 6     # Payload is a message; the next packet is text
    CC_ERROR    = 7     # Some error
    CC_ECHO     = 8     # Echo the incomming header
    CC_NOOP     = 9     # Does nothing, but does get ACKed
    CC_CONFIG   = 10    # Set configuration
    CC_AXES     = 11    # Set configuration for an axis
    CC_INFO     = 12    # Send back info messages about state and condition
    CC_EMPTY    = 13    # Queue is empty, nothing to do.
    CC_DONE     = 20    # A Movement command is finished.
    CC_RMOVE    = 21    # a normal movement segment
    CC_AMOVE    = 22  # a normal movement segment
    CC_JMOVE    = 23  # a normal movement segment

    def __init__(self, seq, code, crc=0):

        self.seq = seq  # Gets set when sent
        self.code = code
        self.crc = crc

        self.acked = None  # Set to true after ACK is recieved, or False if Nacked

        self.ack = None  # ACK header
        self.nack = None  # Nack header

        self.payload = None

    @staticmethod
    def unpack(data):
        seq, code, crc = struct.unpack(CommandHeader.msg_fmt, data)

        o = CommandHeader(seq, code, crc)

        return o

    def _pack(self, crc=0):

        msg = [self.seq, self.code, crc]

        # Build the packet without the CRC as zero
        try:
            return struct.pack(self.msg_fmt, *msg)
        except:
            print("Failed to build struct for :", msg)
            raise

    def encode(self):

        if self.payload:
            try:
                d = self.payload.encode('ascii')  # Guess it is a string
            except AttributeError:
                try:
                    d = self.payload.encode()
                except AttributeError:
                    d = self.payload
        else:
            d = bytearray()

        p = self._pack(0) + d
        crc = crc8(p)

        p = self._pack(crc) + d

        return cobs.encode(p) + bytearray([0])

    @classmethod
    def decode(cls, d):

        b = cobs.decode(d)
        d = bytearray(b)

        h = cls.unpack(d[:cls.size])

        d[3] = 0  # CRC is calculated with a zero here.
        that_crc = crc8(d)
        if h.crc != that_crc:
            raise CRCError(f"CRC Check failed: {h.crc} != {that_crc} ")

        h.payload = d[cls.size:]

        return h

    def __hash__(self):
        return hash((self.code, self.seq, self.crc))

    def __eq__(self, other):
        return (
                self.crc == other.crc and
                self.code == other.code and
                self.seq == other.seq)

    def __str__(self):
        return f"<Header #{self.seq} code={self.code} ({self.crc})> "




class MoveCommand(object):

    msg_fmt = ('<' +
               'I' + # segment_time
               '6i')  # steps

    size = struct.calcsize(msg_fmt)

    def __init__(self, code: int,  x: List[int], t:float=0):
        self.x = [int(e) for e in x] + [0]*(6-len(x))
        self.t = int(round(t * TIMEBASE)) # Convert to integer microseconds

        if code not in (CommandHeader.CC_AMOVE, CommandHeader.CC_RMOVE,
                        CommandHeader.CC_JMOVE):
            raise BadMoveCodeError("Bad Code {}".format(code))

        self.header = CommandHeader(seq=0, code=code)

    @property
    def seq(self):
        return self.header.seq

    @seq.setter
    def seq(self, v):
        self.header.seq = v

    def encode(self):

        self.header.payload = self.pack()

        return self.header.encode()

    def pack(self):
        try:

            return struct.pack(self.msg_fmt, self.t, *self.x)
        except:
            print(self.__dict__)
            raise


    def __repr__(self):
        return f"<AxisSegment {self.axis_n} {self.steps} >"


class AxisConfig(object):
    msg_fmt = ('<' +
               '4B'+  # axis, mode, step, dir, enable
               'I' +  # v_max
               'I'    # a_max
               )

    size = struct.calcsize(msg_fmt)

    def __init__(self, axis_num: int, step_pin: int, direction_pin: int, enable_pin: int,
                 v_max: int, a_max: int):

        self.axis_num = axis_num
        self.mode = 0
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.enable_pin = enable_pin
        self.v_max = int(v_max)
        self.a_max = int(a_max)

        self.header = CommandHeader(seq=0, code=CommandHeader.CC_AXES)

    @property
    def seq(self):
        return self.header.seq

    @seq.setter
    def seq(self, v):
        self.header.seq = v


    def encode(self):

        self.header.payload = struct.pack(self.msg_fmt, self.axis_num,
                                   self.step_pin, self.direction_pin, self.enable_pin,
                                   self.v_max, self.a_max)

        return self.header.encode()

class ConfigCommand(object):

    msg_fmt = ('<' +
               'B' + # n_axes
               'B' + # interrupt_delay
               'B' + # enable_active
               'B' + # debug_print
               'B'   # debug_tick
               )

    size = struct.calcsize(msg_fmt)

    def __init__(self, n_axes: int, itr_delay: int, enable_active: bool=True,
                 debug_print: bool=False, debug_tick: bool=False):
        self.n_axes = n_axes
        self.itr_delay = itr_delay
        self.enable_active = enable_active
        self.debug_print = debug_print
        self.debug_tick = debug_tick

        self.header = CommandHeader(seq=0, code=CommandHeader.CC_CONFIG)

    @property
    def seq(self):
        return self.header.seq

    @seq.setter
    def seq(self, v):
        self.header.seq = v

    def encode(self):

        self.header.payload = struct.pack(self.msg_fmt, self.n_axes, self.itr_delay,
                                          self.enable_active, self.debug_print, self.debug_tick)

        return self.header.encode()


class CurrentState(object):

    msg_fmt = ('<' +
               'i' + # queue_length
               'I' + # queue_time
               '6i' # stepper_postitions
               '6i'  # planner_postitions
               )

    size = struct.calcsize(msg_fmt)

    def __init__(self, b=None):

        if b:
            self.queue_length, self.queue_time, *positions = struct.unpack(self.msg_fmt, b)
        else:
            self.queue_length, self.queue_time, *positions = 0, 0, []

        self.positions, self.planner_positions = positions[:6], positions[6:]


    def __str__(self):
        return f"[ ql={self.queue_length} qt={self.queue_time} {self.positions} ]"