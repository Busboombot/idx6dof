#!/usr/bin/python

import rospy, time, sys, signal
from pyudev import Context
from serial import Serial
from std_msgs.msg import String

def query(ser, q):
	ser.write(q + '\r\n')
	resp = ser.readline()
	if len(resp) >= 2:
		return resp[:-2]
	ser.flushInput()
	ser.flushOutput()
	return resp
	
def send(ser, msg):
	print msg
	ser.flushInput()
	ser.flushOutput()
	ser.write(msg + '\r\n')

class ArmCommunicator:

	currentCommand = None
	keyValues = None
	speed = None
	maxSpeed = None
	
	updateRate = None
	ticker = None
	
	robot = None
	context = None
	
	def interpretCommand(self, cmd):
		keyValues = [0]*17
		for c in cmd:
			cL = c.lower()
			if 97 <= ord(cL) <= 123:
				cI = ord(cL) - 97
				keyValues[cI] = 1 if c.isupper() else -1
		self.keyValues = keyValues
		self.speed = 35 if keyValues[0] == -1 else 70 if keyValues[0] == 0 else 100
		
	def makeCommand(self):
		jointMotion = ''
		kv = self.keyValues
		if kv[16] == 1: #Axis Mode
			print 'Doing high-tech inverse kinematics!'
		else: #Joint Mode
			jointMotion = 't ' + str(int(2000/self.updateRate)) #'t' command takes milliseconds.
			for i in range(2, 8):
				jointMotion += " "+str(kv[i]*self.speed)
		self.currentCommand = jointMotion
				speed = 35 if keyValues[0] == -1 else 70 if keyValues[0] == 0 else 100	
		print 'Current command: %s'%jointMotion
	
	def sendCommand(self):
		send(self.robot, self.currentCommand)
	
	def commandReceiver(self, data):
		self.interpretCommand(data.data)
		self.sendCommand()
		self.makeCommand()
		
	def loop(self):
		while not rospy.is_shutdown():
			self.sendCommand()
			self.ticker.sleep()
		print 'Interrupted. Sending stop signal.'
		self.currentCommand = 's'
		self.sendCommand()
		
	def findRobot(self):
		ignored = []
		while self.robot == None and not rospy.is_shutdown():
			devices = self.context.list_devices(subsystem='tty', ID_BUS='usb')
			for device in devices:
				dn = device.device_node
				if dn in ignored:
					continue
				ser = Serial(dn, timeout=0.1, baudrate=115200)
				time.sleep(2)
				resp = query(ser, 'id')
				if resp == 'stepper':
					self.robot = ser
					print 'Found robot at %s'%dn
					send(ser, 'ms ' + str(self.maxSpeed))
					return
				elif resp != '':
					ignored += dn
				ser.close()
			print 'Did not find the robot, trying again...'
			time.sleep(2)
		print 'Interrupted'	
		
	def __init__(self):
		rospy.init_node('armCommunicator', anonymous=False)
		rospy.Subscriber('controllerCommands', String, self.commandReceiver)
		self.context = Context()
		self.speed = 0
		self.maxSpeed = 500
		self.currentCommand = 's'
		self.updateRate = 4 #Per second.
		self.ticker = rospy.Rate(self.updateRate) #Per second.
		
	
ac = ArmCommunicator()
ac.findRobot()
ac.loop()
