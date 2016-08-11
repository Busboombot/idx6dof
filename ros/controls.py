import select, sys, signal, time
from pyudev import Context
from serial import Serial

class RobotControl:

	usedPorts=None
	
	contSer = None
	roboSer = None

	maxSpeed = None
	speed = None
	
	updateRate = None
	readRate = None
	
	keyValues = None
	currentCommand = None
	
	context = Context()
	
	def query(self, ser, q):
		ser.write(q + '\r\n')
		resp = ser.readline()
		if len(resp) >= 2:
			return resp[:-2]
		ser.flushInput()
		ser.flushOutput()
		return resp

	def send(self, ser, msg):
		ser.flushInput()
		ser.flushOutput()
		ser.write(msg + '\r\n')

	def findDevices(self):
		for device in self.context.list_devices(subsystem='tty', ID_BUS='usb'):
			nde = device.device_node
			if nde in self.usedPorts:
				continue
			ser = Serial(nde, timeout=0.1, baudrate=115200)
			time.sleep(3)
			resp = self.query(ser, 'id')
			if self.contSer == None and resp == 'pendant':
				self.contSer = ser
				self.send(ser, 'ao')
				print "Found controller at " + nde
			elif self.roboSer == None and resp == 'stepper':
				self.roboSer = ser
				print "Found robot control at " + nde
				self.send(ser, "ms 15000")
			else:
				continue
			self.usedPorts.append(nde)

	def readController(self):
		ctr = self.contSer
		if ctr.inWaiting():
			cmd = ctr.readline()
			keyValues = [0]*17
			for c in cmd:
				cL = c.lower()
				if 97 <= ord(cL) <= 123:
					cI = ord(cL) - 97
					keyValues[cI] = 1 if c.isupper() else -1
			self.keyValues = keyValues
			self.speed = 35 if keyValues[0] == -1 else 70 if keyValues[0] == 0 else 100
			return True
		return False
			
	def interpretCommand(self):
		jointMotion = ''
		kv = self.keyValues
		if kv[16] == 1: #Axis Mode
			print 'Doing high-tech inverse kinematics!'
		else: #Joint Mode
			jointMotion = 'm ' + str(self.speed)
			for i in range(2, 7):
				jointMotion += " "+str(kv[i]*self.maxSpeed/self.updateRate)
		self.currentCommand = jointMotion
		
	def sendCommands(self):
		self.send(self.roboSer, self.currentCommand)
		
	def run(self):
		try:
			while True:
				self.findDevices()
				if self.roboSer == None or self.contSer == None:
					continue
				
				if self.readController():
					self.interpretCommand()
					self.sendCommands()
				elif (time.time()*self.updateRate)%1 < 1.0/self.updateRate:
					self.sendCommands()
				time.sleep(self.readRate)
		except KeyboardInterrupt:
			self.send(self.roboSer, 'm 0 0 0 0 0 0 0') #To avoid terrible accidents.
			print 'Interrupted.'

	def __init__(self):
		self.usedPorts = []
		self.keyValues = [0]*17
		self.currentCommand = ""
		self.maxSpeed = 700
		self.speed = 0
		self.updateRate = 4 #Per second
		self.readRate = 0.05 #Delay in seconds
	
ctrl = RobotControl()
ctrl.run()
	
