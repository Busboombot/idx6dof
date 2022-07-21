from __future__ import print_function

import logging
from time import time

import serial

from .messages import *
import queue

import serial
import sys
import time
from cobs import cobs
import struct

import selectors

logger = logging.getLogger('message')

# These parameters must match those in the  firmware,
# in idx_stepper.h
TIMEBASE = 1_000_000  # microseconds
N_BIG = 2 ** 32 - 1  # ULONG_MAX
fp_bits = 8  # Bits in the fraction portion of the floating point representation

TERMINATOR = b'\0'

from dataclasses import dataclass


class TimeoutException(Exception):
    pass


class ProtocolException(Exception):
    pass


def _message_callback(proto, m):
    pl = m.payload.decode('utf8')

    if m.code == CommandHeader.CC_DEBUG:
        logger.debug(pl)
    elif m.code == CommandHeader.CC_ERROR:
        logger.error(pl)
    else:
        logger.info(pl)


class SyncProto(object):

    def __init__(self,
                 stepper_port, encoder_port=None, stepper_baud=115200, encoder_baud=115200,
                 message_callback = None, timeout=.1):

        self.step_ser = serial.Serial(stepper_port, baudrate=stepper_baud, timeout=timeout)

        if encoder_port is not None:
            self.enc_ser = serial.Serial(encoder_port, baudrate=encoder_baud, timeout=timeout)

        if message_callback is None:
            message_callback = _message_callback

        self.message_callback = message_callback

        self.empty = True
        self.seq = 0;
        self.last_ack = -1;
        self.last_done = -1;
        self.current_state = CurrentState()
        self.timeout = timeout

        self.sel = selectors.DefaultSelector()

        self.sel.register(self.step_ser, selectors.EVENT_READ, self.read_stepper_message)
        self.sel.register(self.enc_ser, selectors.EVENT_READ, self.read_encoder_message)

    def close(self):

        self.sel.close()
        self.step_ser.close()
        if self.enc_ser:
            self.enc_ser.close()


    @property
    def queue_length(self):
        return float(self.current_state.queue_length)

    @property
    def queue_time(self):
        return float(self.current_state.queue_time) / 1e6

    def read_stepper_message(self, ser):
        data = ser.read_until(TERMINATOR)

        if data:
            m = CommandHeader.decode(data[:-1])
            self.handle_stepper_message(m)
            return m
        else:
            return None


    def handle_stepper_message(self, m):

        m.recieve_time = time.time()

        if m.code in (CommandHeader.CC_ACK, CommandHeader.CC_NACK, CommandHeader.CC_ECHO):
            self.last_ack = m.seq
            return

        elif m.code in ( CommandHeader.CC_ERROR, CommandHeader.CC_MESSAGE):
            if self.message_callback:
                self.message_callback(self, m)
            else:
                pass
            return

        elif m.code == CommandHeader.CC_DONE:
            self.current_state = CurrentState(m.payload)
            if self.queue_length == 0:
                self.empty = 0;
            self.last_done = m.seq

        elif m.code == CommandHeader.CC_EMPTY:
            self.empty = True;

    def read_encoder_message(self,ser):
        data = ser.read_until(TERMINATOR)

        if data:
            m =  EncoderMessage.decode(data[:-1])
            self.handle_encoder_message(m)
            return m
        else:
            return None

    def handle_encoder_message(self,m):
        pass

    def select(self, timeout=False):

        if timeout is False:
            timeout = self.timeout

        messages = []
        events =self.sel.select(timeout)
        for key, mask in events:
            f, ser = key.data, key.fileobj
            m = f(ser)
            messages.append(m)

        return messages

    def __iter__(self):

        while True:
            e = self.select(self.timeout)
            if e:
                yield from e
            else:
                break



    def read(self):
        """Read a single byte at a time, return a message when it is complete"""

        return self.read_message()

    def read_message(self):
        """Read until the terminator character."""

        data = self.step_ser.read_until(TERMINATOR)

        if data:
            m = CommandHeader.decode(data[:-1])
            self.handle_stepper_message(m)
            return m
        else:
            return None

    @property
    def queue_length(self):
        return self.current_state.queue_length

    @property
    def queue_time(self):
        return self.current_state.queue_time


    # Sending messages to the stepper controller
    #

    def send(self, m):

        m.send_time = time.time()
        self.seq += 1
        m.seq = self.seq

        b = m.encode()

        self.step_ser.write(b)

        # Read until we get the ack
        while True:
            self.read()
            if self.last_ack == m.seq:
                break

        return m

    def send_command(self, c):
        self.send(CommandHeader(seq=self.seq, code=c))

    def config(self, itr_delay: int = 4, enable_active=True,
               debug_print: bool = False, debug_tick: bool = False, segment_complete_pin=12,
               axes: List[AxisConfig] = []):

        # Send the top level config, to set the number of
        # axes
        self.send(ConfigCommand(len(axes), itr_delay, segment_complete_pin,
                                enable_active, debug_print, debug_tick))

        # Then send the config for each axis.
        for ac in axes:
            self.send(ac)

    def _move(self, code: int, x: List[int], t=0):
        m = MoveCommand(code, x, t=t)

        m.done = False

        self.current_state.queue_length += 3;

        self.send(m)
        self.empty = False;

    def amove(self, x: List[int]):
        """Absolute position move"""
        self._move(CommandHeader.CC_AMOVE, x, t=0)

    def rmove(self, x: List[int]):
        "Relative position move"
        self._move(CommandHeader.CC_RMOVE, x, t=0)

    def jog(self, t: float, x: List[int]):
        """Jog move. A jog move replaces the last move on the (step generator side)
        planner, then becomes a regular relative move. """
        self._move(CommandHeader.CC_JMOVE, x, t=t)

    def run(self):
        self.send_command(CommandHeader.CC_RUN)

    def stop(self):
        self.send_command(CommandHeader.CC_STOP)

    def info(self):
        self.send_command(CommandHeader.CC_INFO)

    def reset(self):
        self.send_command(CommandHeader.CC_RESET)

    def zero(self):
        self.enc_ser.write(b'z')
        self.send_command(CommandHeader.CC_ZERO)
