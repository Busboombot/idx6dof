from __future__ import print_function

import struct
from typing import List
from dataclasses import dataclass
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
    CC_DONE     = 3    # A Movement command is finished.
    CC_EMPTY    = 4  # Queue is empty, nothing to do.

    CC_RMOVE    = 11   # a normal movement segment
    CC_AMOVE    = 12  # a normal movement segment
    CC_JMOVE    = 13  # a normal movement segment

    CC_RUN      = 21
    CC_STOP     = 22
    CC_RESET    = 23
    CC_ZERO    = 24
    CC_CONFIG   = 25  # Set configuration
    CC_AXES     = 26  # Set configuration for an axis


    CC_MESSAGE  = 91     # Payload is a message; the next packet is text
    CC_ERROR    = 92     # Some error
    CC_ECHO     = 93     # Echo the incomming header
    CC_DEBUG    = 94  # Echo the incomming header
    CC_INFO     = 95  # Send back info messages about state and condition
    CC_NOOP = 99  # Does nothing, but does get ACKed

    cmap = {
        CC_ACK: "ACK",
        CC_NACK: "NACK",
        CC_DONE: "DONE",  # A Movement command is finished
        CC_EMPTY: "EMPTY",  # Queue is empty, nothing to do.

        CC_RMOVE: "RMOVE",  # A relative movement segment, with just the relative distance.
        CC_AMOVE: "AMOVE",  # An absolute movement
        CC_JMOVE: "JMOVE",  # A Jog movement.

        CC_RUN: "RUN",
        CC_STOP: "STOP",
        CC_RESET: "RESET",  #
        CC_ZERO: "ZERO",  # Zero positions
        CC_CONFIG: "CONFIG",  # Reset the configuration
        CC_AXES: "AXES",  # Configure an axis

        CC_MESSAGE: "MESSAGE",  # Payload is a message; the next packet is text
        CC_ERROR: "ERROR",  # Some error
        CC_ECHO: "ECHO",  # Echo the incomming header
        CC_DEBUG: "DEBUG",  #
        CC_INFO: "INFO",  # Return info messages

        CC_NOOP: "NOOP",  # Does nothing, but get ACKED
    }

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
        return f"< #{self.seq} {self.cmap[self.code]} > "

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
               'B' + # Segment Complete Pin
               'B' + # enable_active
               'B' + # debug_print
               'B'   # debug_tick
               )

    size = struct.calcsize(msg_fmt)

    def __init__(self, n_axes: int, itr_delay: int,segment_complete_pin:int = 12,
                 enable_active: bool=True, debug_print: bool=False, debug_tick: bool=False):

        self.n_axes = n_axes
        self.itr_delay = itr_delay
        self.enable_active = enable_active
        self.debug_print = debug_print
        self.debug_tick = debug_tick
        self.segment_complete_pin = segment_complete_pin

        self.header = CommandHeader(seq=0, code=CommandHeader.CC_CONFIG)

    @property
    def seq(self):
        return self.header.seq

    @seq.setter
    def seq(self, v):
        self.header.seq = v

    def encode(self):

        self.header.payload = struct.pack(self.msg_fmt, self.n_axes, self.itr_delay, self.segment_complete_pin,
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
        return f"[ l{self.queue_length} t{self.queue_time} {self.positions} ]"


encoder_msg_fmt = (
        '6c' +  # limit_states[6]
        'c' +  # code
        'c' +  # pad
        '6i'  # Positions[6]
)

ls_map = {
    2: 'HL',
    1: 'LH',
    3: 'HH',
    0: 'LL'
}

code_map = {
    1: 'L', # Triggered on limit
    2: 'Z', # Triggered from zero command
    3: 'S', # Triggered from end of segment
}


@dataclass
class EncoderMessage:
    """Class for keeping track of an item in inventory."""
    limit: int
    code: int
    direction: int
    position: float

    def __repr__(self):
        return (f'E[{code_map[self.code]} {ls_map[self.limit]} {"<" if self.direction else ">"} {self.position}]')

    @classmethod
    def decode(cls, data):

        encoders = []

        try:
            v = struct.unpack(encoder_msg_fmt, cobs.decode(data))

            for l, c, p in zip(v[:6], v[6], v[-6:]):
                ls = int.from_bytes(l, "big")

                encoders.append(EncoderMessage(ls & 3, c, (ls & 4) >> 2, p))

        except Exception as e:
            raise

        return encoders

