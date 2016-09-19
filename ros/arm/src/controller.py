#!/usr/bin/python

import rospy, time, sys, signal
from pyudev import Context
from serial import Serial
from arm.msg import Switches, SwitchPos

publisher = rospy.Publisher('controllerCommands', Switches, queue_size=50)

def query(ser, q):
	ser.write(q + '\r\n')
	resp = ser.readline()
	if len(resp) >= 2:
		return resp[:-2]
	ser.flushInput()
	ser.flushOutput()
	return resp
	
def send(ser, msg):
	ser.flushInput()
	ser.flushOutput()
	ser.write(msg + '\r\n')
	
def publishCommand(cmd):
	publisher.publish(cmd)

class Controller:

	context = None

	controller = None
	
	
	def findController(self):
		ignored = []
		while self.controller == None and not rospy.is_shutdown():
			devices = self.context.list_devices(subsystem='tty', ID_BUS='usb')
			for device in devices:
				dn = device.device_node
				if dn in ignored:
					continue
				ser = Serial(dn, timeout=0.1, baudrate=115200)
				time.sleep(2)
				resp = query(ser, 'id')
				if resp == 'pendant':
					self.controller = ser
					print 'Found controller at %s'%dn
					return
				elif resp != '':
					ignored += dn
				ser.close()
			print 'Did not find a controller, trying again...'
			time.sleep(2)
		print 'Interrupted'
		
	def makeMSG(self, cmd):
		values = [None]*17
		msg = Switches
		for c in cmd:
			cL = c.lower()
			switch = SwitchPos()
			if 97 <= ord(cL) <= 123:
				cI = ord(cL) - 97
				values[cI] = Switches.UP if c.isupper() else Switches.DOWN
		for i in range(len(values)):
			if values[i] == None:
				values[i] = Switches.NEUTRAL
		msg.pos = values
		return msg
				
	def setupController(self):
		send(self.controller, 'ao')
				
	def setup(self):
		self.context = Context()
		self.findController()
		self.setupController()
		
	def loop(self):
		while not rospy.is_shutdown():
			while self.controller.inWaiting():
				line = self.controller.readline()
				if line != '':
					publishCommand(makeMSG(line[:-2]))
			time.sleep(0.1)
		print 'Interrupted, sending stop command.'
		publishCommand('s')
		self.controller.close()
		

rospy.init_node("controllerReader", anonymous=False)
contr = Controller()
contr.setup()
contr.loop()
