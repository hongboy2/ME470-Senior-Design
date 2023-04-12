#!/usr/bin/env python

########################
# Imports
########################
import time
import RPi.GPIO as GPIO
import math

###########################
# Constants
###########################
#Directions of rotation from perspective with motor shaft pointed towards you with right hand rule
POSITIVE = True         #counterclockwise
NEGATIVE = False          #clockwise
#Modes of movement
ABSOLUTE = False         #rotate relative to the 0 position
RELATIVE = True          #rotate relative to the current position

#####################################################################################
# Stepper Main Class
#####################################################################################
class Stepper_Driver(object):

	#------------------------------------------
	# Variable Defaults
	#------------------------------------------
	_pinStep = -1              # gpio pin for step signal
	_pinDir = -1               # gpio pin for direction signal
	_pinEn = -1                # gpio pin for enable signal
	_direction = None          # current direction of rotation for positive steps
	_enabled = None            # motor can move when true
	_movementMode = None       # mode of movement
	_active = False            # the moving state of the motor
	_stopped = False           # whether or not to stop the motor
	_pulsesPerStep = 256       # microstepping - number of pulses for one full step
	_stepsPerRev = 200         # steps per one revolution of the motor
	_pulseCountAbs = 0         # current position of stepper in steps
	_pulseCountAbsTarget = 0   # the target position in steps
	_pulseSpeed = -1           # the current speed in pulses per second
	_pulseSpeedTarget = -1     # the maximum speed in pulses per second
	_pulseAcceleration = -1    # the acceleration in steps per second per second
	_pulseAccelerationMax = -1 # the max acceleration in steps per second per second
	_pulseAccelerationMin = -1 # the min acceleration in steps per second per second
	_pulseTimeLast = 0         # the last step time in microseconds
	_pulseAccelCount = 0       # step counter
	_pulsePeriod = -1          # the current interval between two steps
	_pulsePeriod0 = 200        # initial step size in microseconds
	_pulsePeriodMax = -1       # maximium recognizable period by driver circuit
	_pulsePeriodMin = -1       # minimum allowed pulse width in microseconds
	_pulsePeriodLast = 0       # last step size in microseconds
	_pulsePeriodTarget = -1    # min step size in microseconds based on maxSpeed

	#----------------------------------------------
	# constructor
	#----------------------------------------------
	def __init__(self, pinStep, pinDir, pinEn, pulsesPerStep, stepsPerRev, accel0, accelMax, accelMin, speed0, speedMax, speedMin):
		self._pinStep = pinStep
		self._pinDir = pinDir
		self._pinEn = pinEn
		self._pulsesPerStep = pulsesPerStep
		self._stepsPerRev = stepsPerRev
        self._pulseAcceleration = accel0
        self._pulseAccelerationMax = accelMax
        self._pulseAccelerationMin = accelMin
        self._pulsePeriod0 = 1/speed0
   		self._pulsePeriodMin = 1/speedMax
        self._pulsePeriodMax = 1/speedMin
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self._pinStep, GPIO.OUT)
		GPIO.setup(self._pinDir, GPIO.OUT)
		GPIO.setup(self._pinEn, GPIO.OUT)

	#-----------------------------------------------------------------------
	# destructor
	#-----------------------------------------------------------------------
	def __del__(self):
		self.setEnabled(False)
		GPIO.cleanup()

	#-----------------------------------------------------------------------
	# returns the motor's pin for step signal
	#-----------------------------------------------------------------------
	def getPinStep(self):
		return self._pinStep

	#-----------------------------------------------------------------------
	# overwrites the motor's pin for step signal
	#-----------------------------------------------------------------------
	def setPinStep(self, stepPin):
		self._stepPin = stepPin

	#-----------------------------------------------------------------------
	# returns the motor's pin for direction signal
	#-----------------------------------------------------------------------
	def getPinDir(self):
		return self._pinDir

	#-----------------------------------------------------------------------
	# overwrites the motor's pin for direction signal
	#-----------------------------------------------------------------------
	def setPinDir(self, pinDir):
		self._pinDir = pinDir

	#-----------------------------------------------------------------------
	# returns the motor's pin for enable signal
	#-----------------------------------------------------------------------
	def getPinEn(self):
		return self._pinEn

	#-----------------------------------------------------------------------
	# overwrites the motor's pin for enable signal
	#-----------------------------------------------------------------------
	def setPinEn(self, pinEn):
		self._pinEn = pinEn

	#-----------------------------------------------------------------------
	# returns the motor shaft direction
	#-----------------------------------------------------------------------
	def getDirection(self):
		return (self._direction)

	#-----------------------------------------------------------------------
	# sets the motor shaft direction
	#-----------------------------------------------------------------------
	def setDirection(self, direction):
		self._direction = direction
		GPIO.output(self._pinDir, direction)
	#-----------------------------------------------------------------------
	# gets whether motor is enabled or disabled
	#-----------------------------------------------------------------------
	def getEnabled(self, en):
		return self._enabled

	#-----------------------------------------------------------------------
	# enables or disables the motor current output
	#-----------------------------------------------------------------------
	def setEnabled(self, en):
		self._enabled = en

	#-----------------------------------------------------------------------
	# gets whether motor is enabled or disabled
	#-----------------------------------------------------------------------
	def getPowered(self):
		return self._powered

	#-----------------------------------------------------------------------
	# enables or disables the motor current output
	#-----------------------------------------------------------------------
	def setPowered(self, powered):
		self._powered = powered
		GPIO.output(self._pinEn, not powered)

	#-----------------------------------------------------------------------
	# get the current general movment mode: False = Absolute; True = Relative
	#-----------------------------------------------------------------------
	def getMovementMode(self):
		return self._movementMode

	#-----------------------------------------------------------------------
	# set the general movment mode: False = Absolute; True = Relative
	#-----------------------------------------------------------------------
	def setMovementMode(self, movementMode):
		self._movementMode = movementMode

	#-----------------------------------------------------------------------
	# gets whether motor is actively moving
	#-----------------------------------------------------------------------
	def getActive(self):
		return self._active

	#-----------------------------------------------------------------------
	# gets whether motor is seeking a target or just moving at a speed
	#-----------------------------------------------------------------------
	def getSeeking(self):
		return self._seeking

	#-----------------------------------------------------------------------
	# get the stopped status of the motor
	#-----------------------------------------------------------------------
	def getStopped(self):
		return self._stopped

	#-----------------------------------------------------------------------
	# set the stopped status of the current movement
	#-----------------------------------------------------------------------
	def setStopped(self, stop):
		self._stopped = stop

	#-----------------------------------------------------------------------
	# get the microstep setting of the motor
	#-----------------------------------------------------------------------
	def getPulsesPerStep(self):
		return self._pulsesPerStep

	#-----------------------------------------------------------------------
	# set the number of pulses to make one full step (microstepping)
	#-----------------------------------------------------------------------
	def setPulsesPerStep(self, pulses):
		self._pulsesPerSec = pulses

	#-----------------------------------------------------------------------
	# get the number of steps per revolution
	#-----------------------------------------------------------------------
	def getStepsPerRev(self):
		return self._stepsPerRev

	#-----------------------------------------------------------------------
	# set the number of steps requires to rotate the motor shaft 360 degrees
	#-----------------------------------------------------------------------
	def setStepsPerRev(self, steps):
		self._stepsPerRev = steps

	#-----------------------------------------------------------------------
	# returns the current motor position in microsteps
	#-----------------------------------------------------------------------
	def getPulseCountAbs(self):
		return self._pulseCountAbs

	#-----------------------------------------------------------------------
	# overwrites the current motor position in microsteps
	#-----------------------------------------------------------------------
	def setPulseCountAbs(self, pos):
		self._pulseCountAbs = pos

	#-----------------------------------------------------------------------
	# returns the current motor position in microsteps
	#-----------------------------------------------------------------------
	def getRevCountAbs(self):
		return self._pulseCountAbs*self._pulsesPerStep*self._stepsPerRev

	#-----------------------------------------------------------------------
	# overwrites the current motor position in revolutions
	#-----------------------------------------------------------------------
	def setRevCountAbs(self, count):
		self.setPulseCountAbs(self._pulsesPerStep*self._stepsPerRev*count)

	#-----------------------------------------------------------------------
	# returns the target motor speed in pulses per second
	#-----------------------------------------------------------------------
	def getPulseSpeedTarget(self):
		return self._pulseSpeedTarget

	#-----------------------------------------------------------------------
	# sets the target speed in pulses per second
	#-----------------------------------------------------------------------
	def setPulseSpeedTarget(self, speed):
		# make sure speed is positive
		if (speed < 0.0):
			speed = -speed
		# only change speed if it is a different value
		if (self._pulseSpeedTarget != speed):
			#store new value in pulses/sec
			self._pulseSpeedTarget = speed
			# adjust the minimum pulse period which is what creates the fastest speed (1 sec / (speed(pulses/sec)) * 1000000 microseconds/second)
			self._pulsePeriodTarget = 1000000.0 / speed
			# Recompute pulse count for acceleration from current speed and adjust speed if accelerating or cruising
			if (self._pulseAccelCount > 0):
				self._pulseAccelCount = (self._pulseSpeed * self._pulseSpeed) / (2.0 * self._pulseAcceleration) # Equation 16
				self.computeNewSpeed()

	#-----------------------------------------------------------------------
	# sets the target speed in revolutions per second
	#-----------------------------------------------------------------------
	def setRevSpeedTarget(self, revSpeed):
        self.setPulseSpeedTarget(revSpeed*self._stepsPerRev*self._pulsesPerStep)

	#-----------------------------------------------------------------------
	# returns the motor acceleration/decceleration in pulses per sec per sec
	#-----------------------------------------------------------------------
	def getPulseAcceleration(self):
		return self._pulseAcceleration

	#-----------------------------------------------------------------------
	# sets the motor acceleration/decceleration in pulses per sec per sec
	#-----------------------------------------------------------------------
	def setPulseAcceleration(self, acceleration):
		if (acceleration == 0.0):
			return
		if (acceleration < 0.0):
			acceleration = -acceleration
		if (self._pulseAcceleration != acceleration):
			# Recompute _pulseAccelCount to be proportional for this acceleration
			self._pulseAccelCount = self._pulseAccelCount * (self._pulseAcceleration / acceleration)
			# New pulsePeriod0 per Equation 7, with correction per Equation 15
			self._pulsePeriod0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
			self._pulseAcceleration = acceleration
			self.computeNewSpeed()

	#-----------------------------------------------------------------------
	# runs the motor to the given position.
	# with acceleration and deceleration
	# blocks the code until finished or stopped from a different thread!
	# returns true when the movement if finshed normally and false,
	# when the movement was stopped
	#-----------------------------------------------------------------------
	def runToPulseCount(self, pulses):
		self._active = True
		self._seeking = True
		if(self._movementMode == RELATIVE):
			self._pulseCountAbsTarget = round(self._pulseCountAbs + pulses)
		else:
			self._pulseCountAbsTarget = round(pulses)
		self._pulsePeriod = 0
		self._pulseSpeed = 0.0
		self._pulseAccelCount = 0
		self.computeNewSpeed()
		while (self.run() and not self._stopped and self._powered and self._enabled): #returns false when target position is reached
			pass
		self._active = False
		self._seeking = False
		return not self._stopped

	#-----------------------------------------------------------------------
	# runs the motor to the given position.
	# with acceleration and deceleration
	# blocks the code until finished or stopped from a different thread!
	# returns true when the movement if finshed normally and false,
	# when the movement was stopped
	#-----------------------------------------------------------------------
	def runToRevCount(self, rev):
		return self.runToPulseCount(rev*self._pulsesPerStep*self._stepsPerRev)

	#-----------------------------------------------------------------------
	# calculates a new speed if a speed was made
	# returns true if the target position is reached
	# should not be called from outside!
	#-----------------------------------------------------------------------
	def run(self):
		if (self.runSpeed()): #returns true, when a step is made
			self.computeNewSpeed()
		return (self._pulseSpeed != 0.0 and self.pulsesToGo() != 0)


	#-----------------------------------------------------------------------
	# returns the remaining distance the motor should run
	#-----------------------------------------------------------------------
	def pulsesToGo(self):
		return self._pulseCountAbsTarget - self._pulseCountAbs

	#-----------------------------------------------------------------------
	# this methods does the actual steps with the current speed
	#-----------------------------------------------------------------------
	def runSpeed(self):
		# Dont do anything unless we actually have a step interval
		if (not self._pulsePeriod):
			return False
		t = time.time_ns()/1000
		if (t - self._pulseTimeLast >= self._pulsePeriod):
			if (self._direction == POSITIVE):
				self._pulseCountAbs += 1
			else:
				self._pulseCountAbs -= 1
			self.pulse()
			self._pulseTimeLast = t # Caution: does not account for costs in step()
			return True
		else:
			return False


	#-----------------------------------------------------------------------
	# returns the calculated current speed depending on the acceleration
	# this code is based on:
	# "Generate stepper-motor speed profiles in real time" by David Austin
	#
	# https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
	# https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
	#-----------------------------------------------------------------------
	def computeNewSpeed(self):
		pulsesToGo = self.pulsesToGo() # get how many pulses are left until target
		pulsesToStop = (self._pulseSpeed * self._pulseSpeed) / (2.0 * self._pulseAcceleration) # Equation 16
		# check if at target position with pulses needed to stop less than 1
		if (pulsesToGo == 0 and pulsesToStop <= 1):
			# We are at the target and its time to stop
			self._pulsePeriod = 0
			self._pulseSpeed = 0.0
			self._pulseAccelCount = 0
			return
		# check if positive motion is needed
		if (pulsesToGo > 0):
			# check if accelerating
			if (self._pulseAccelCount > 0):
				# check if stopping at this rate will take more pulses than getting to the target or moving in the wrong direction
				if ((pulsesToStop >= pulsesToGo) or self._direction == NEGATIVE):
					# start decelerating
					self._pulseAccellCount = -pulsesToStop
			# check if decelerating
			elif (self._pulseAccelCount < 0):
				# check if stopping at this rate will stop before getting to the target and moving in the right direction
				if ((pulsesToStop < pulsesToGo) and self._direction == POSITIVE):
					# start accelerating
					self._pulseAccelCount = -self._pulseAccelCount
		# check if negative motion is needed
		elif (pulsesToGo < 0):
			# check if accelerating
			if (self._pulseAccelCount > 0):
				# check if stopping at this rate will take more pulses than getting to the taret or moving in the wrong direction
				if ((pulsesToStop >= -pulsesToGo) or self._direction == POSITIVE):
					#start decelerating
					self._pulseAccelCount = -pulsesToStop
			# check if decelerating
			elif (self._pulseAccelCount < 0):
				# check if stopping at this rate will stop before getting to the target and moving in the right direction
				if ((pulsesToStop < -pulsesToGo) and self._direction == NEGATIVE):
					# start accelerating
					self._pulseAccelCount = -self._pulseAccelCount
		# Check if this is the first pulse from stopped
		if (self._pulseAccelCount == 0):
			# set period to starting period
			self._pulsePeriod = self._pulsePeriod0
			# make sure step pin is low so a pulse is effective
			GPIO.output(self._pinStep, GPIO.LOW)
			# set direction to start moving ihttp://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.61n
			if(pulsesToGo < 0):
				self.setDirection(NEGATIVE)
			elif(pulsesToGo > 0):
				self.setDirection(POSITIVE)
		# not first pulse from stoped
		else:
			self._pulsePeriod = self._pulsePeriod - ((2.0 * self._pulsePeriod) / ((4.0 * self._pulseAccelCount) + 1)) # Equation 13
			self._pulsePeriod = max(self._pulsePeriod, self._pulsePeriodTarget)
		self._pulseAccelCount += 1
		# store period for next loop
		self._pulsePeriodLast = self._pulsePeriod
		# set the speed magnitude according to the new period
		self._pulseSpeed = 1000000.0 / self._pulsePeriod #1 second in nano seconds / period in nanoseconds
		# set the speed sign according the direction of motion
		if (self._direction == NEGATIVE):
			self._pulseSpeed = -self._pulseSpeed

	#-----------------------------------------------------------------------
	# method that makes one pulse
	# for the TMC2209 there needs to be a signal duration of minimum 100 ns
	#-----------------------------------------------------------------------
	def pulse(self):
		GPIO.output(self._pinStep, GPIO.HIGH)
		time.sleep(1/1000/1000)
		GPIO.output(self._pinStep, GPIO.LOW)
		time.sleep(1/1000/1000)

##############################
	#-----------------------------------------------------------------------
	# runs the motor to the given speed.
	# with acceleration and deceleration
	# blocks the code until finished or stopped from a different thread!
	# returns true when the movement if finshed normally and false,
	# when the movement was stopped
	#-----------------------------------------------------------------------
	def runToPulseSpeed(self, pulseSpeed):
		self._active = True
		self._pulsePeriod = 0
		self._pulseSpeed = 0.0
		self._pulseAccelCount = 0
		self._pulseSpeedTarget = pulseSpeed
		self.convergeSpeed()
		while (not self._seeking and not (self._targetPulseSpeed == 0 and self._pulseSpeed == 0) and not self._stopped and self._powered and self._enabled): #returns false when target position is reached
			if (self.runSpeed()): #returns true, when a step is made
				self.convergeSpeed()
			pass
		self._active = False
		return not self._stopped

	#-----------------------------------------------------------------------
	# runs the motor to the given position.
	# with acceleration and deceleration
	# blocks the code until finished or stopped from a different thread!
	# returns true when the movement if finshed normally and false,
	# when the movement was stopped
	#-----------------------------------------------------------------------
	def runToRevSpeed(self, revSpeed):
		return self.runToPulseSpeed(revSpeed*self._pulsesPerStep*self._stepsPerRev)

	#-----------------------------------------------------------------------
	# returns the calculated current speed depending on the acceleration
	# this code is based on:
	# "Generate stepper-motor speed profiles in real time" by David Austin
	#
	# https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
	# https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
	#-----------------------------------------------------------------------
	def convergeSpeed(self):
		# check if positive accelleration is needed
		if (self._pulseSpeed > self._pulseSpeedTarget):
			self._pulseAccellCount = self._pulseAccellCount - 1
		# check if negative accelleration is needed
		if (self._pulseSpeed > self._pulseSpeedTarget):
			self._pulseAccellCount = self._pulseAccellCount + 1
		# Check if this is the first pulse from stopped
		if (self._pulseAccelCount == 0):
			# set period to starting period
			self._pulsePeriod = self._pulsePeriod0
			# make sure step pin is low so a pulse is effective
			GPIO.output(self._pinStep, GPIO.LOW)
			# set direction to start moving ihttp://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.61n
			if(pulsesToGo < 0):
				self.setDirection(NEGATIVE)
			elif(pulsesToGo > 0):
				self.setDirection(POSITIVE)
		# not first pulse from stoped
		else:
			self._pulsePeriod = self._pulsePeriod - ((2.0 * self._pulsePeriod) / ((4.0 * self._pulseAccelCount) + 1)) # Equation 13
			self._pulsePeriod = max(self._pulsePeriod, self._pulsePeriodTarget)
		self._pulseAccelCount += 1
		# store period for next loop
		self._pulsePeriodLast = self._pulsePeriod
		# set the speed magnitude according to the new period
		self._pulseSpeed = 1000000.0 / self._pulsePeriod #1 second in nano seconds / period in nanoseconds
		# set the speed sign according the direction of motion
		if (self._direction == NEGATIVE):
			self._pulseSpeed = -self._pulseSpeed