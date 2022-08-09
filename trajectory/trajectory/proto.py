import logging
import selectors
import time
from time import time
from collections import deque
import serial
from typing import Union, Tuple, List, Any, Dict

from .messages import *

logger = logging.getLogger('message')

# These parameters must match those in the  firmware,
# in idx_stepper.h
TIMEBASE = 1_000_000  # microseconds
N_BIG = 2 ** 32 - 1  # ULONG_MAX
N_AXES = 6
fp_bits = 8  # Bits in the fraction portion of the floating point representation

TERMINATOR = b'\0'

from dataclasses import dataclass


class TimeoutException(Exception):
    pass


class ProtocolException(Exception):
    pass


def _message_callback(proto, m):
    try:
        pl = m.payload.decode('utf8')
    except:
        pl = m.payload

    if m.code == CommandCode.DEBUG:
        logger.debug(pl)
    elif m.code == CommandCode.ERROR:
        logger.error(pl)
    else:
        logger.info(pl)


@dataclass
class AxisState():
    spos: int = None  # Stepper position
    epos: int = None  # Encoder position
    hl_limit: int = None
    lh_limit: int = None
    last_limit: int = None
    direction: int = None

    def __str__(self):
        d = '+' if self.direction else '-'
        return f"<AS {d} {self.spos}/{self.epos} hl{self.hl_limit} lh{self.lh_limit}"


class SyncProto(object):

    def __init__(self,
                 stepper_port, encoder_port=None, stepper_baud=115200, encoder_baud=115200,
                 message_callback=None, timeout=.1):

        self.step_ser = serial.Serial(stepper_port, baudrate=stepper_baud, timeout=timeout)

        if encoder_port is not None:
            self.enc_ser = serial.Serial(encoder_port, baudrate=encoder_baud, timeout=timeout)
        else:
            self.enc_ser = None

        if message_callback is None:
            message_callback = _message_callback

        self.message_callback = message_callback

        self.timeout = timeout

        self.encoder_multipliers = [1] * N_AXES

        self.empty = True
        self.seq = 0;
        self.last_ack = -1;
        self.last_done = -1;

        self.running = False

        self._reset_states()

        self.sel = selectors.DefaultSelector()

        self.sel.register(self.step_ser, selectors.EVENT_READ, self.read_stepper_message)
        if self.enc_ser:
            self.sel.register(self.enc_ser, selectors.EVENT_READ, self.read_encoder_message)

        self.queue = deque(maxlen=100)

    def _reset_states(self):

        self.axis_state = [AxisState() for _ in range(N_AXES)]

        self.current_state = CurrentState()
        self.encoder_state = [None] * N_AXES

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

        try:
            if data:
                m = CommandHeader.decode(data[:-1])
                self.handle_stepper_message(m)
                return m
            else:
                return None
        except Exception as e:
            print("ERROR", e)
            return None

    def handle_stepper_message(self, m):

        m.recieve_time = time()

        if m.code in (CommandCode.ACK, CommandCode.NACK, CommandCode.ECHO):
            self.last_ack = m.seq
            return

        elif m.code in (CommandCode.ERROR, CommandCode.MESSAGE):

            if self.message_callback:
                self.message_callback(self, m)
            else:
                pass
            return

        elif m.code in (CommandCode.DONE, CommandCode.EMPTY, CommandCode.ZERO):

            self.current_state = CurrentState(m.payload)

            for p, ax in zip(self.current_state.positions, self.axis_state):
                ax.spos = p

            if m.code == CommandCode.EMPTY:
                self.empty = True;
            elif m.code == CommandCode.DONE:
                self.last_done = m.seq

    def read_encoder_message(self, ser):
        data = ser.read_until(TERMINATOR)

        if data:
            try:
                m = EncoderReport.decode(data[:-1])
                self.handle_encoder_message(m)
                return m
            except Exception as e:
                print(e, data)
                return None
        else:
            return None

    def handle_encoder_message(self, m):
        self.encoder_state = m.encoders

        for i, (es, mult, ax) in enumerate(zip(self.encoder_state, self.encoder_multipliers, self.axis_state)):

            ax.epos = int(round(es.position * mult))

            if i == m.axis_code:

                if es.limit_code == LimitCode.LH:
                    ax.lh_limit = ax.last_limit = ax.epos

                elif es.limit_code == LimitCode.HL:
                    ax.hl_limit = ax.last_limit = ax.epos

    def update(self, timeout=False):
        '''Read all outstanding messages, handle them, and add them to the queue,'''

        if timeout is False:
            timeout = self.timeout

        messages = []
        events = self.sel.select(timeout)
        for key, mask in events:
            f, ser = key.data, key.fileobj
            m = f(ser)
            if m and not m.is_ack:
                self.queue.append(m)

        return len(events)

    def iupdate(self, timeout=False):
        t = time()
        while True:
            self.update()
            yield from self

            if timeout is not False and time() > t + timeout:
                break

    def __iter__(self):

        while True:
            try:
                yield self.queue.popleft()
            except IndexError:
                return

    def runempty(self, cb=None, timeout=False):

        if not self.running:
            self.run()

        while not self.empty:
            self.update(timeout)
            for m in self:
                if cb:
                    cb(self, m)

    def runout(self, cb=None, timeout=False):

        if not self.running:
            self.run()

        while self.update(timeout):
            for m in self:
                if cb:
                    cb(self, m)

    @property
    def queue_length(self):
        return self.current_state.queue_length

    @property
    def queue_time(self):
        return self.current_state.queue_time

    # Sending messages to the stepper controller
    #

    def send(self, m):

        m.send_time = time()
        self.seq += 1
        m.seq = self.seq

        b = m.encode()

        self.step_ser.write(b)

        # Read until we get the ack
        while True:
            self.update();
            if self.last_ack == m.seq:
                break

        return m

    def send_command(self, c):
        self.send(CommandHeader(seq=self.seq, code=c))

    def config(self, itr_delay: int = 4, segment_complete_pin=0, limit_hit_pin=0,
               debug_print: bool = False, debug_tick: bool = False,
               axes: List[AxisConfig] = []):

        # Send the top level config, to set the number of
        # axes
        self.send(ConfigCommand(len(axes), itr_delay, segment_complete_pin, limit_hit_pin,
                                debug_print, debug_tick))

        self.axes = axes

        # Then send the config for each axis.
        for ac in axes:
            self.send(ac)

    def _move(self, code: int, x: Union[List[Any], Tuple[Any], Dict], t=0):

        # Convert a dict-based move into an array move.
        if isinstance(x, dict):
            x_ = [0] * len(self.axes)
            for k,v in x.items():
                assert isinstance(k, int)
                x_[k] = v
            x = x_

        m = MoveCommand(code, x, t=t)

        m.done = False

        self.current_state.queue_length += 3;

        self.send(m)
        self.empty = False;

    def amove(self, x: Union[List[Any], Tuple[Any], Dict]):
        """Absolute position move"""
        self._move(CommandCode.AMOVE, x, t=0)

    def rmove(self, x: Union[List[Any], Tuple[Any], Dict]):
        "Relative position move"
        self._move(CommandCode.RMOVE, x, t=0)

    def hmove(self, x: Union[List[Any], Tuple[Any], Dict]):
        "A homing move, which will stop when it gets to a limit. "
        self._move(CommandCode.HMOVE, x, t=0)

    def jog(self, t: float, x: Union[List[Any], Tuple[Any], Dict]):
        """Jog move. A jog move replaces the last move on the (step generator side)
        planner, then becomes a regular relative move. """
        self._move(CommandCode.JMOVE, x, t=t)

    def run(self):
        self.running = True
        self.send_command(CommandCode.RUN)

    def stop(self):
        self.running = False
        self.send_command(CommandCode.STOP)

    def info(self):
        self.send_command(CommandCode.INFO)

    def reset(self):
        self.runout()
        self._reset_states()
        self.send_command(CommandCode.RESET)
        self.runout()

    def zero(self):
        self.runout()
        self._reset_states()
        if self.enc_ser:
            self.enc_ser.write(b'z')
        self.send_command(CommandCode.ZERO)
        self.runout()

    def pollEncoders(self):
        self.enc_ser.write(b'p')

        for m in self.iupdate(1):
            if m.name == 'POLL':
                return m
