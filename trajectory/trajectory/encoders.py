
import serial
from cobs import cobs
import struct
usb_port = '/dev/cu.usbmodem1415131'
baud = 115200
ser = serial.Serial(usb_port, baud, timeout=10);

msg_fmt = (
    '<6i' +  # position[6]
    '6h' +  # hl_limits[6]
    '6h'    # lh_limits[6]
  )

while True:

    buf = b""

    while True:
        d = ser.read()

        if not d or ord(d) == 0:
            break

        buf += d

    try:


        v = struct.unpack(msg_fmt, cobs.decode(buf))

        print (list(zip(v[0:6], v[6:12], v[12:])))

    except cobs.DecodeError as e:
        print(e, len(buf), struct.calcsize(msg_fmt))
        print(":".join("{:02x}".format(ord(c)) for c in buf))
    except struct.error as e:
        print(e)



