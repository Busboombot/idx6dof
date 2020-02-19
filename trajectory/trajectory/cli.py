


def run_joystick():
    """Read the joystick and write the vaules to a file file"""
    import sys
    from time import sleep
    from .joystick import PygameJoystick as Joystick

    f=None

    if len(sys.argv) > 1:
        with open(sys.argv[1], 'w'):
            for values in Joystick(.5):
                f.write(','.join(str(e) for e in values))
                f.write('\n')
                f.seek(0)
                sleep(.2)
    else:
        for values in Joystick(.5):
            print(values)
            

def run_joy_planner():

    from trajectory import Proto, Command, Response, SimSegment,  \
                           SegmentList, SegmentIterator, SegmentBuffer, \
                           Joystick
    from time import time, sleep

    def get_joy():
        
        while True:
            with open('/tmp/joystick') as f:
                yield [float(e) for e in f.readline().split(',')]

    with Proto('/dev/cu.usbmodemFD1431') as proto:
    
        last_time = time()
        last_velocities = [0]*6
        seq = 0
    
        for velocities in get_joy():
            
            axis_mode = velocities.pop(0)

            dt = time()-last_time

            if dt >= .20 and len(proto)<=2:
                last_time = time()

                x = [ .5*(v0+v1)*dt  for v0, v1 in zip(last_velocities, velocities) ]
  
                msg = Command(seq, 10, dt*1e6, last_velocities, velocities, x)
               
                proto.write(msg)

                seq += 1
            
                last_velocities = velocities
            elif dt < .20:
                sleep(.20-dt)

