import serial
ser = serial.Serial('/dev/cu.usbmodemFD1411', 115200)

s = set()

while True:
    v =  ser.readline().strip()
    s.add(int(v))
    print sorted(s)
    