#!/usr/bin/python3 

# Project: Python Library for 7Bot Robotic Arm  ( Version 2.0 ) 
# Author: Artyom Liu & Jerry Peng
# Date:   July 23th, 2020

import serial 


BAUD_RATE = 115200      # baud rate of robot serial port
SERVO_NUM = 7           # servo motor number of robot

# Register ID 
# ROM
DEVICE_TYPE_ID = 0
VERSION_ID = 1
MAC_ID = 2
#EEPROM
EEPROM_LEN = 9
EEPROM_ID = 11
DEVICE_ID = 11
BAUDRATE_ID = 12
OFFSET_ID = 13
#RAM 
EEPROM_LOCK_ID = 28
MOTOR_STATUS_ID = 29
EFFECTOR_ID = 30
VACUUM_ID = 31
SPEED_ID = 32
TIME_ID = 39
ANGLE_ID = 46
END_LENGTH_ID = 53
IK_ID = 54
IK_DATA_LENGTH = 9
IK7_DATA_LENGTH = 12
ANGLE_FEEDBACK_FREQ_ID = 82
ANGLE_FEEDBACK_ID = 83
LOAD_FEEDBACK_FREQ_ID = 90
LOAD_FEEDBACK_ID = 91

class Arm7Bot: 
	""" The interface on host machine with 7Bot robot """

	def __init__(self, port): 
		self.ser = serial.Serial(port, BAUD_RATE, timeout = 0.2) 

	def readReg(self, addr, num): 
		# asking for register information 
		buf = [ 0x03, addr & 0xff, num & 0xff ] 
		self.writeSerial(buf) 
		
		ret_pack = self.readSerial()
		if (ret_pack.pop(0) != 0x03):  
			raise serial.SerialException("mismatching pack type") 
		return ret_pack[2:] 
		
	def writeReg(self, addr: int, data: list): 
		buf = [ 0x04, addr & 0xff, len(data) & 0xff ] 
		for d in data: 
			buf.append(d & 0xff) 
		self.writeSerial(buf) 
		# information feedback from robot is not required so far 
	
	def readAnglesFb(self): 
		ret_pack = self.readSerial()
		if (ret_pack.pop(0) != 0x05):  
			raise serial.SerialException("mismatching pack type") 
		if(ret_pack.pop(0) != ANGLE_FEEDBACK_ID):
			raise serial.SerialException("mismatching pack type") 
		return ret_pack[1:] 
	
	
	# Get Functions #

	# get decice code
	def getDeviceCode(self):
		return self.readReg(DEVICE_TYPE_ID, 1)[0]

	# get version
	def getVersion(self):
		return self.readReg(VERSION_ID, 1)[0]/10
	
	# get MAC address(lenmgth: 6 bytes)
	def getMAC(self):
		 return self.listToString(self.readReg(MAC_ID, 6))
	# Function to convert   
	def listToString(self, l: list):  
		# initialize an empty string 
		str1 = ""  
		# traverse in the string   
		for ele in l:  
			str1 += hex(ele)[2:]
		# return string   
		return str1  

	# get ID
	def getID(self):
		return self.readReg(DEVICE_ID, 1)[0]

	# get offsets. return joints offset, qualified range:[-16, 16]
	def getOffsets(self):
		offsets = []
		tmp = self.readReg(OFFSET_ID, SERVO_NUM)
		for offset in tmp: offsets.append(offset-128)
		return offsets

	# get angle. return joint angle, range:[0, 180]
	def getAngle(self, ID: int):
		return self.readReg(ANGLE_FEEDBACK_ID+ID, 1)

	# get angles. return joints angle, range:[0, 180]
	def getAngles(self):
		angles = []
		tmp = self.readReg(ANGLE_FEEDBACK_ID, SERVO_NUM)
		for angle in tmp: angles.append(angle)
		return angles

	# get load. return joint load, range:[-30, 30]
	def getLoad(self, ID: int):
		return self.readReg(LOAD_FEEDBACK_ID+ID, 1)[0]-128

	# get angles. return joints angle, range:[0, 180]
	def getLoads(self):
		loads = []
		tmp = self.readReg(LOAD_FEEDBACK_ID, SERVO_NUM)
		for load in tmp: loads.append(load-128)
		return loads

	# Set Functions #

	# set decice code
	def setID(self, ID: int):
		self.writeReg(DEVICE_ID, [ID])

	# set joints offsets
	def setOffsets(self, offsets: list):
		self.setLock(0)
		offsetSendData = []
		for tmp in offsets: offsetSendData.append(tmp+128)
		self.writeReg(OFFSET_ID, offsetSendData)
		self.setLock(1)

	# set EEPROM write lock: 0-lock off(enable writting), 1-lock on(protect EEPROM)
	def setLock(self, lock: int):
		self.writeReg(EEPROM_LOCK_ID, [lock])
	
	# set arm force status: 0-protection, 1-servo, 2-forceless
	def setStatus(self, status: int):
		self.writeReg(MOTOR_STATUS_ID, [status])

	# set end-effector type: 0-vacuum, 1-claw
	def setEffector(self, type: int):
		self.writeReg(EFFECTOR_ID, [type])

	# set vacuum status: 0-turn off， 1-turn on
	def setVacuum(self, vacuum: int):
		self.writeReg(VACUUM_ID, [vacuum])

	# set motion speed
    # Speed: angular speed of joints motion ( unit: 1.9°/s, range: [0, 100] )
    # if Speed = 0; it means set joints motion speed to the maximum, i.e. 190°/s
	def setSpeed(self, speed: int):
		speeds = [speed, speed, speed, speed, speed, speed, speed]
		self.writeReg(SPEED_ID, speeds)

	# set motion execute time 
    # Time:  motion execute time ( unit:100ms, range: [0, 100] )
	def setTime(self, time: int):
		times = [time, time, time, time, time, time, time]
		self.writeReg(TIME_ID, times)

	# set individual joint 
    # ID: joint ID ( range [0, 6]  )
    # angle: joint angle ( unit: degree, range: [0, 180] )
	def setAngle(self, ID: int, angle: int):
		self.writeReg(ANGLE_ID+ID, [angle])

	# set 7 joints angle (Unit:degree) at once, range: [0, 180]
	def setAngles(self, angles: list):
		self.writeReg(ANGLE_ID, angles)

	# Set robot position use IK parameters. Function: IK6, 
    # input joint[6] & Vector56(joint[5] to joint[6] direction), calculate theta[0]~[4].
	def setIK6(self, j6: list, vec56: list):
		dataIK = [(j6[0]+1024)>>8, (j6[0]+1024)%256, 
				  (j6[1]+1024)>>8, (j6[1]+1024)%256, 
				  (j6[2]+1024)>>8, (j6[2]+1024)%256,  
				  vec56[0]+128, vec56[1]+128, vec56[2]+128
				 ]
		# print(dataIK)
		self.writeReg(IK_ID, dataIK)

	# Set robot position use IK parameters. Function: IK7,
    # input joint[6], Vector56(joint[5] to joint[6] direction) & Vector67(joint[6] to joint[7]), calculate theta[0]~[5].
	def setIK7(self, j6: list, vec56: list, vec67: list):
		dataIK = [(j6[0]+1024)>>8, (j6[0]+1024)%256, 
				  (j6[1]+1024)>>8, (j6[1]+1024)%256, 
				  (j6[2]+1024)>>8, (j6[2]+1024)%256,  
				  vec56[0]+128, vec56[1]+128, vec56[2]+128,
				  vec67[0]+128, vec67[1]+128, vec67[2]+128
				 ]
		self.writeReg(IK_ID, dataIK)

	# set joints' angle auto feedback frequency
	# Freq: frequency of joints' angle feedback. (unit: Hz, range: [0~50])
	def setAnglesFbFreq(self, freq: int):
		self.writeReg(ANGLE_FEEDBACK_FREQ_ID, [freq])

	# set joints' load auto feedback frequency
	# Freq: frequency of joints' load feedback. (unit: Hz, range: [0~50])
	def setLoadsFbFreq(self, freq: int):
		self.writeReg(LOAD_FEEDBACK_FREQ_ID, [freq])

	# EEPROM data init, this function will erase offset data.
	def EEPROMinit(self):
		data = [0, 0, 128, 128, 128, 128, 128, 128, 128]
		self.setLock(0)
		self.writeReg(EEPROM_ID, data)
		self.setLock(1)

	def readSerial(self): 
		cnt = 0 
		# read pack head 
		while (True): 
			tmp = self.ser.read() 
			#print(tmp) 
			if (tmp == b'\xaa'): 
				tmp = self.ser.read() 
				if (tmp == b'\x77'): 
					break 
			cnt += 1 
			if (50 == cnt): 
				raise serial.SerialTimeoutException() 
		
		# from here data is to be returned 
		tmp = self.ser.read(3) 
		ret = [ 0xaa, 0x77, tmp[0], tmp[1], tmp[2] ]
		tmp = self.ser.read(ret[4] + 2) 
		for d in tmp: ret.append(d) 
		crc = self.CRC16_MODBUS(ret[0:-2]) 
		if ((crc & 0xff == ret[-2]) and ((crc >> 8) & 0xff == ret[-1])): 
			return ret[2:-2] 
		else: 
			# print(ret)
			raise serial.SerialException("data corrupted") 

	def writeSerial(self, data: list): 
		buf = [ 0xaa, 0x77 ] 
		for tmp in data: buf.append(tmp) 

		crc = self.CRC16_MODBUS(buf) 
		buf.append(crc & 0xff) 
		buf.append((crc >> 8) & 0xff) 

		return self.ser.write(bytes(buf)) == len(buf) 

	def invert8(self, val): 
		ret = i = 0 
		while (i < 8): 
			ret |= ((val >> i) & 0x01) << (7 - i) 
			i += 1 
		return ret & 0xff 

	def invert16(self, val): 
		ret = i = 0 
		while (i < 16): 
			ret |= ((val >> i) & 0x01) << (15 - i) 
			i += 1 
		return ret & 0xffff 

	def CRC16_MODBUS(self, data: list): 
		wCRCin = 0xffff 
		wCPoly = 0x8005

		for tmp in data: 
			tmp = self.invert8(tmp) 
			wCRCin ^= (tmp << 8) & 0xffff
			i = 0 
			while (i < 8): 
				i += 1 
				if ((wCRCin & 0x8000) != 0): 
					wCRCin = 0xffff & ((wCRCin << 1) & 0xffff) ^ wCPoly 
				else: 
					wCRCin = (wCRCin << 1) & 0xffff 
		wCRCin = self.invert16(wCRCin) 
		return wCRCin 

