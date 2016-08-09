#!/usr/bin/python

import select, rospy, sys, signal, os
from serial import Serial
 
from pyudev import Context

def keyInterrupt(signal, frame):
	sys.exit(0)
	
signal.signal(signal.SIGINT, keyInterrupt)
signal.signal(signal.SIGTSTP, keyInterrupt)

def controllerMaster():
	rospy.init_node('masterController', anonymous=False)
	timeoutTicks = 45
	r = rospy.Rate(4)

	serials = {}
	
	disqualifiedDevices = set()
	
	context = Context()
	
	pollster = select.poll()
	

	while not rospy.is_shutdown():
	
		for device in context.list_devices(subsystem='tty', ID_BUS='usb'):
		
			if (device.device_node not in disqualifiedDevices) :
			
				disqualifiedDevices.add(device.device_node)
			
				serialAccess = Serial(device.device_node, timeout=10)
				
				fd = open (device.device_node,'r')
				
				serials[fd.fileno()] = (serialAccess, fd)
				
				pollster.register(fd.fileno(), select.POLLIN)
				
				print 'Found and added device at {}'.format(device.device_node)
				
		poll = pollster.poll(1000)
		
		for fd, event in poll:
			if not fd in serials:
				print "Nope!", fd, event, serials.keys()
				continue
				
			serial = serials[fd]

			if event== 1:
				while True:
					print serial[0].readline()
				
		r.sleep()

controllerMaster()
