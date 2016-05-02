
# ----------------------------------------------------------------------------
#  Motordriver.py: 3. PI, Uni Stuttgart
#  Copyright (C) 2014  Stefan Lasse
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
# ----------------------------------------------------------------------------

import time
import serial
import numpy

class Motordriver():
  
  """ Gives an interface to the Motordriver. """
  
  ON        = "1"
  OFF       = "0"
  
  STEPS     = "steps"
  DEGREE    = "deg"
  PI        = "pi"
  
  RELATIVE  = "REL"
  ABSOLUTE  = "ABS"
  
  CW        = "CW"
  CCW       = "CCW"
  STOP      = "STOP"
  
  SLOW      = "0"
  FAST      = "1"
  MIXED     = "2"
  
  DIM       = "1"
  BRIGHT    = "2"
  
  MENUESCAPE    = "0"
  ONDESELECTED  = "1"
  ONSELECTED    = "2"
  OFFDESELECTED = "3"
  OFFSELECTED   = "4"
  
  
  # to keep some internal information
  __subSteps = [0.0, 0.0, 0.0, 0.0]
  __stepsPerFullRotation = [0.0, 0.0, 0.0, 0.0]
  __gearRatio = [0.0, 0.0, 0.0, 0.0]
  
  
  # --------------------------------------------------------------------------
  # linux: interface='/dev/ttyUSB0'
  # windows: interface='COM3'
  def __init__(self, interface='/dev/ttyUSB0'):
    self.ser = serial.Serial(
      port = interface,
      baudrate = 57600,
      bytesize = serial.EIGHTBITS,
      parity = serial.PARITY_NONE,
      stopbits = serial.STOPBITS_ONE,
      timeout = 5,      
      xonxoff = 0,
      rtscts = 0,
      dsrdtr = 0,
      writeTimeout = 5
    )
    self.ser.close()
    self.ser.open()
    time.sleep(0.5)
    
    # get some internal information
    for i in range(4):
      self.__subSteps[i] = self.getSubSteps(i)
      self.__stepsPerFullRotation[i] = self.getStepsPerFullRotation(i)
      self.__gearRatio[i] = self.getGearRatio(i)
    
    
  # --------------------------------------------------------------------------
  def __del__(self):
    # stop all motor movements when closing the interface
    self.stopAllMovements()
    if self.ser.isOpen():
      self.ser.close()
    
  # --------------------------------------------------------------------------
  # helpers
  # --------------------------------------------------------------------------
  
  # --------------------------------------------------------------------------
  def __checkMotor(self, motor):
    if not isinstance(motor, int):
      print "error: motor number must be an integer."
      return False
    if motor > 3:
      print "maximum motor is 3"
      return False
    if motor < 0:
      print "minimum motor is 0"
      return False
    
    return True
  
  # --------------------------------------------------------------------------
  def __checkUnit(self, unit):
    if unit == self.STEPS or unit == self.DEGREE or unit == self.PI:
      return True
    else:
      return False
  
  # --------------------------------------------------------------------------
  def __degreeToSteps(self, deg, i):
    return int(round(deg * (( self.__stepsPerFullRotation[i]   \
                             *self.__gearRatio[i]              \
                             *self.__subSteps[i])/(360.0))))
    
  # --------------------------------------------------------------------------
  def __radianToSteps(self, rad, i):
    return int(round(rad * (( self.__stepsPerFullRotation[i]   \
                             *self.__gearRatio[i]              \
                             *self.__subSteps[i])/(2.0))))
  
  # --------------------------------------------------------------------------
  # low level functionality
  # --------------------------------------------------------------------------
  
  # --------------------------------------------------------------------------
  def __sendCommand(self, cmd):    
    ack = "A"
    maxtries = 0
    self.ser.flushOutput()
    
    while ord(ack) != 6 and maxtries < 10:
      self.ser.write(cmd + '\n')
      self.ser.flush()
      time.sleep(0.05)
      maxtries += 1
      while self.ser.inWaiting() == 0:
        pass
      
      if self.ser.inWaiting() > 0:
        ack = self.ser.read(1)
        
    if maxtries == 9:
      print "error: unable to send command: " + cmd
    
    self.ser.flushOutput()
    return
  
  # --------------------------------------------------------------------------
  def __readResponse(self):
    out = ""    
    while self.ser.inWaiting() > 0:
      c = self.ser.read(1)
      if c != '\n':
        out += c
    
    self.ser.flushInput()
    return out.decode("utf-8")
    
  # --------------------------------------------------------------------------
  # command implementation
  # --------------------------------------------------------------------------

  # --------------------------------------------------------------------------
  def reset(self):
    self.__sendCommand("STOPALL")
    self.__sendCommand("*RST")
    return
  
  # --------------------------------------------------------------------------
  def getState(self, mot):
    if self.__checkMotor(mot):
      self.__sendCommand("GETMOTSTATE " + str(mot))
      return str(self.__readResponse())

  # --------------------------------------------------------------------------
  def getIDN(self):
    self.__sendCommand("*IDN?")
    return str(self.__readResponse())
  
  # --------------------------------------------------------------------------
  def setIDN(self, id):
    if not isinstance(id, basestring):
      print "ID is not a string."
      return
    
    if len(id) > 20:
      print "IDN too long. max: 20 characters"
      return
    
    if len(id) == 0:
      print "IDN too short. min: 1 character"
      return
    
    self.__sendCommand("*IDN " + id)
    return
  
  # --------------------------------------------------------------------------
  def goHome(self):
    self.moveToAbsolutePosition(0, 0, self.STEPS)
    self.moveToAbsolutePosition(1, 0, self.STEPS)
    self.moveToAbsolutePosition(2, 0, self.STEPS)
    self.moveToAbsolutePosition(3, 0, self.STEPS)
    
  # --------------------------------------------------------------------------
  def moveToAbsolutePosition(self, motor=0, pos=0, unit=DEGREE):
    if unit is self.STEPS and not isinstance(pos, int):
      print "err: unit is STEPS, so position must be an integer"
      return
    
    if pos < 0:
      print "position must be a positive value"
      return
    
    if self.__checkMotor(motor) and self.__checkUnit(unit):
      cmd = "MOVEABS " + str(motor) + " " + str(pos) + " " + unit
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def moveRelative(self, motor=0, pos=0, unit=DEGREE):
    if unit is self.STEPS and not isinstance(pos, int):
      print "err: unit is STEPS, so position must be an integer"
      return
    
    if self.__checkMotor(motor) and self.__checkUnit(unit):
      cmd = "MOVEREL " + str(motor) + " " + str(pos) + " " + unit
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def zeroMotor(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ZERORUN " + str(motor))
    return
    
  # --------------------------------------------------------------------------
  def turnOnMotor(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ENABLE " + str(motor) + " " + self.ON)
    return

  # --------------------------------------------------------------------------
  def turnOffMotor(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ENABLE " + str(motor) + " " + self.OFF)

  # --------------------------------------------------------------------------
  def getPosition(self, motor=0, unit=DEGREE):
    if self.__checkMotor(motor) and self.__checkUnit(unit):    
      self.__sendCommand("GETPOS " + str(motor) + " " + unit)
      return float(self.__readResponse())
    return

  # --------------------------------------------------------------------------
  def saveCurrentConfiguration(self):
    self.__sendCommand("SAVECONF")
    return
  
  # --------------------------------------------------------------------------
  def loadSavedConfiguration(self):
    self.__sendCommand("LOADCONF")
    return
  
  # --------------------------------------------------------------------------
  def isMoving(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ISMOVING " + str(motor))
      resp = self.__readResponse()
      if resp == "1":
        return True
      else:
        return False
    return
    
  # --------------------------------------------------------------------------
  def isConnected(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ISCON " + str(motor))
      resp = self.__readResponse()
      if resp == "1":
        return True
      else:
        return False
    return

  # --------------------------------------------------------------------------
  # not supported in current firmware version
  def getAnalogValue(self, channel=0):
    if self.__checkMotor(channel):
      self.__sendCommand("GETANALOG " + str(channel))
      return int(self.__readResponse())
    return    
  
  # --------------------------------------------------------------------------
  def getZeroPosition(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETZEROPOS " + str(motor))
      return int(self.__readResponse())
    return
  
  # --------------------------------------------------------------------------
  def setZeroPosition(self, motor=0, position=0):
    if not isinstance(position, int):
      print "zero position must be given in steps, integer."
      return
    
    if self.__checkMotor(motor):
      cmd = "SETZEROPOS " + str(motor) + " " + str(position)
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def getGearRatio(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETGEARRATIO " + str(motor))
      return float(self.__readResponse())
    return
  
  # --------------------------------------------------------------------------
  def setGearRatio(self, motor=0, ratio=60.0/20.0):
    if self.__checkMotor(motor):
      cmd = "SETGEARRATIO " + str(motor) + " " + str(ratio)
      self.__sendCommand(cmd)
      self.__gearRatio[motor] = ratio
    return
    
  # --------------------------------------------------------------------------
  def getStepsPerFullRotation(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETFULLROT " + str(motor))
      return int(self.__readResponse())
    return

  # --------------------------------------------------------------------------
  def setStepsPerFullRotation(self, motor=0, steps=400):
    if not isinstance(steps, int):
      print "must be given as integer number"
      return
    
    if steps != 200 and steps != 400:
      print "steps must be either 200 or 400."
      return
    
    if self.__checkMotor(motor):
      cmd = "SETFULLROT " + str(motor) + " " + str(steps)
      self.__sendCommand(cmd)
      self.__stepsPerFullRotation[motor] = steps
    return
    
  # --------------------------------------------------------------------------
  def getSubSteps(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETSUBSTEPS " + str(motor))
      return int(self.__readResponse())
    return

  # --------------------------------------------------------------------------
  def setSubSteps(self, motor=0, substeps=4):
    if not isinstance(substeps, int):
      print "must be given as integer number"
      return
       
    if substeps < 1 or substeps > 32:
      print "substeps must be a positive number"
      return
    
    if self.__checkMotor(motor):
      cmd = "SETSUBSTEPS " + str(motor) + " " + str(substeps)
      self.__sendCommand(cmd)
      self.__subSteps[motor] = substeps
    return

  # --------------------------------------------------------------------------
  def getWaitTimeBetweenSteps(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETWAITTIME " + str(motor))
      return int(self.__readResponse())
    return

  # --------------------------------------------------------------------------
  def setWaitTimeBetweenSteps(self, motor=0, waittime=3):
    if not isinstance(waittime, int):
      print "waittime must be given as integer number"
      return
    
    if waittime < 1:
      print "waittime must be >= 1"
      return
    
    if self.__checkMotor(motor):
      cmd = "SETWAITTIME " + str(motor) + " " + str(waittime)
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def getMotorCurrent(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETCURR " + str(motor))
      return float(self.__readResponse())
    return
  
  # --------------------------------------------------------------------------
  def setMotorCurrent(self, motor=0, curr=0.5):	  
    if curr > 2.5:
      print "motor current must be <= 2.5 A"
      return
    if curr < 0.0:
      print "motor current must be positive"
      return
    if self.__checkMotor(motor):
      cmd = "SETCURR " + str(motor) + " " + str(curr)
      self.__sendCommand(cmd)
    return

  # --------------------------------------------------------------------------
  def getMotorDecay(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETDECAY " + str(motor))
      return int(self.__readResponse())
    return

  # --------------------------------------------------------------------------
  def setMotorDecay(self, motor=0, decay=SLOW):
    if not (decay == self.SLOW or decay == self.FAST or decay == self.MIXED):
      print "err: unknown decay mode"
      return
    if self.__checkMotor(motor):
      cmd = "SETDECAY " + str(motor) + " " + str(decay)
      self.__sendCommand(cmd)
    return
  
  # --------------------------------------------------------------------------
  def setConstAngularVelocity(self, motor=0, direction=CW, time=10.0):
    if time < 5.0:
      print "time must be >= 5.0 seconds"
      return
      
    if self.__checkMotor(motor):
      cmd = "SETCONSTSPEED " + str(motor) + " " + direction + " " + str(time)
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def factoryReset(self):
    self.__sendCommand("FACTORYRESET")
    return
  
  # --------------------------------------------------------------------------
  def stopAllMovements(self):
    self.__sendCommand("STOPALL")
    return

  # --------------------------------------------------------------------------
  def defineInternalProgramStep(self, stepNo=0, pos=[0.0, 0.0, 0.0, 0.0],
                                unit=DEGREE, mode=RELATIVE):
    # check validity of stepNo, positions, unit and mode
    if len(pos) != 4:
      print "ERROR: position list must have 4 entries"
      return
    
    if stepNo < 0 or stepNo >= 16:
      print "ERROR: step number must be in [0..15]"
      return
    
    if not (unit is self.STEPS or unit is self.DEGREE or unit is self.PI):
      print "ERROR: unit must be either STEPS oder DEGREE or PI"
      return    
    
    # convert position to steps:
    step = [0, 0, 0, 0]
    if unit is self.STEPS: 
      for i in range(4):
        step[i] = int(pos[i])
        
    elif unit is self.DEGREE:
      for i in range(4):
        if abs(pos[i]) > 360.0:
          print "ERROR: position must be <= 360.0 degree"
        else:
          step[i] = self.__degreeToSteps(pos[i], i)
        
    elif unit is self.PI:
      for i in range(4):
        if abs(pos[i] > 2*numpy.pi):
          print "ERROR: position must be <= 2.0*pi"
        else:
          step[i] = self.__radianToSteps(pos[i], i)
          
    # build up serial string
    # here positions are in steps
    cmd = "SETPROGSTEP " + str(stepNo)  + " " \
                         + str(step[0]) + " " \
                         + str(step[1]) + " " \
                         + str(step[2]) + " " \
                         + str(step[3]) + " " \
                         + mode

    # send command
    self.__sendCommand(cmd)

  # --------------------------------------------------------------------------
  def setForbiddenZone(self, motor=0, start=0, stop=0, unit=DEGREE):

    # convert position to steps:
    startSteps = 0
    stopSteps = 0
    if unit is self.STEPS: 
      startSteps = int(start)
      stopSteps = int(stop)
        
    elif unit is self.DEGREE:
      if abs(start) > 360.0 or abs(stop) > 360.0:
        print "ERROR: start and stop must be <= 360.0 degree"
      else:
        startSteps = self.__degreeToSteps(start, motor)
        stopSteps = self.__degreeToSteps(stop, motor)
        
    elif unit is self.PI:
      if abs(start > 2*numpy.pi) or abs(stop > 2*numpy.pi):
        print "ERROR: start and stop must be <= 2.0*pi"
      else:
        startSteps = self.__radianToSteps(start, motor)
        stopSteps = self.__radianToSteps(stop, motor)
    
    if startSteps > stopSteps:
      print "stop value must be larger start value"
      return
    if self.__checkMotor(motor):
      cmd = "SETFORBZONE " + str(motor) + " " + str(startSteps) + " " + str(stopSteps)
      self.__sendCommand(cmd)
    return

  # --------------------------------------------------------------------------
  def turnForbiddenZoneOn(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ENABFORBZONE " + str(motor) + " " + self.ON)
    return

  # --------------------------------------------------------------------------
  def turnForbiddenZoneOff(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ENABFORBZONE " + str(motor) + " " + self.OFF)

  # --------------------------------------------------------------------------
  def display(self, mode=BRIGHT):
    if not (mode == self.OFF or mode == self.DIM or mode == self.BRIGHT):
      print "err: unknown display mode"
      return
    cmd = "DISPLAY " + str(mode)
    self.__sendCommand(cmd)
    return
      
  # --------------------------------------------------------------------------
  def led(self, mode=BRIGHT, color=[0, 0, 0]):
    if not (mode == self.MENUESCAPE or mode == self.ONDESELECTED or mode == self.ONSELECTED or mode == self.OFFDESELECTED or mode == self.OFFSELECTED):
      print "err: unknown LED mode"
      return
    for i in range(3):
      if color[i] > 255:
        print "err: color value must be <= 255"
        return
      if color[i] < 0:
        print "err: color value must be >= 0"
        return
      if not isinstance(color[i], int):
        print "color value must be given as integer number"
        return
    cmd = "LED " + str(mode) + " " + str(color[0]) + " " + str(color[1]) + " " + str(color[2])
    self.__sendCommand(cmd)
    return
      
      

      
      
      