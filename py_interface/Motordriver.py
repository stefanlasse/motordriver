
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

class Motordriver():
  
  """ Gives an interface to the Motordriver. """
  
  ON     = "1"
  OFF    = "0"
  
  STEPS  = "steps"
  DEGREE = "deg"
  PI     = "pi"
  
  CW     = "CW"
  CCW    = "CCW"
  
  # --------------------------------------------------------------------------
  def __init__(self, interface='/dev/ttyUSB0'):
    self.ser = serial.Serial(
    	port = interface,
    	baudrate = 57600,
    	parity = serial.PARITY_NONE,
    	stopbits = serial.STOPBITS_ONE,
    	bytesize = serial.EIGHTBITS
    )
    self.ser.close()
    self.ser.open()
    self.ser.isOpen()
    
  # --------------------------------------------------------------------------
  def __del__(self):
    self.ser.close()
    
  # --------------------------------------------------------------------------
  # helpers
  # --------------------------------------------------------------------------
  
  # --------------------------------------------------------------------------
  def __checkMotor(self, motor):
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
  # low level functionality
  # --------------------------------------------------------------------------
  
  # --------------------------------------------------------------------------
  def __sendCommand(self, cmd):
    
    ack = 'A'
    maxtries = 0
    
    while ord(ack) != 6 and maxtries < 10:
      time.sleep(0.05)
      self.ser.write(cmd + '\n')
      time.sleep(0.05)
      maxtries += 1
      while self.ser.inWaiting() == 0:
        pass
      
      if self.ser.inWaiting() > 0:
        ack = self.ser.read(1)
    
    if maxtries == 9:
      print "unable to send the command " + cmd
    
    return
  
  # --------------------------------------------------------------------------
  def __readResponse(self):
    out = ''    
    while self.ser.inWaiting() > 0:
      c = self.ser.read(1)
      if c != '\n':
        out += c
    
    return
    
  # --------------------------------------------------------------------------
  # command implementation
  # --------------------------------------------------------------------------
  
  # --------------------------------------------------------------------------
  def reset(self):
    # first stop all motor movements
    self.__sendCommand("STOPALL")
    self.__sendCommand("*RST")
    return
    
  # --------------------------------------------------------------------------
  def getIDN(self):
    self.__sendCommand("*IDN?")
    return self.__readResponse()
  
  # --------------------------------------------------------------------------
  def setIDN(self, id):
    if len(id) > 20:
      print "IDN too long. max: 20 characters"
      return
    
    if len(id) == 0:
      print "IDN too short. min: 1 character"
      return
    
    self.__sendCommand("*IDN " + id)
    return
    
  # --------------------------------------------------------------------------
  def moveToAbsolutePosition(self, motor=0, pos=0, unit=DEGREE):
    if self.__checkMotor(motor) and self.__checkUnit(unit):
      cmd = "MOVEABS " + str(motor) + " " + str(pos) + " " + unit
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def moveRelative(self, motor=0, pos=0, unit=DEGREE):
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
      self.__sendCommand("ENABLE " + str(motor) + " " + ON)
    return

  # --------------------------------------------------------------------------
  def turnOffMotor(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ENABLE " + str(motor) + " " + OFF)

  # --------------------------------------------------------------------------
  def getPosition(self, motor=0, unit=DEGREE):
    if self.__checkMotor(motor) and self.__checkUnit(unit):    
      self.__sendCommand("GETPOS " + str(motor) + " " + unit)
      return self.__readResponse()
    return

  # --------------------------------------------------------------------------
  def saveCurrentConfiguration(self):
    pass
  
  # --------------------------------------------------------------------------
  def loadSavedConfiguration(self):
    pass
  
  # --------------------------------------------------------------------------
  def isMoving(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("ISMOVING " + str(motor))
      resp = self.__readResponse()
      if resp == '1':
        return True
      else:
        return False
    return

  # --------------------------------------------------------------------------
  def getAnalogValue(self):
    pass
  
  # --------------------------------------------------------------------------
  def getOpticalZeroPosition(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETOPTZEROPOS " + str(motor))
      return self.__readResponse() + " steps"
    return
  
  # --------------------------------------------------------------------------
  def setOpticalZeroPosition(self, motor=0, position=0):
    if self.__checkMotor(motor):
      cmd = "SETOPTZEROPOS " + str(motor) + " " + str(position)
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def getGearRatio(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETGEARRATIO " + str(motor))
      return self.__readResponse()
    return
  
  # --------------------------------------------------------------------------
  def setGearRatio(self, motor=0, ratio=60.0/18.0):
    if self.__checkMotor(motor):
      cmd = "SETGEARRATIO " + str(motor) + " " + str(ratio)
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def getStepsPerFullRotation(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETFULLROT " + str(motor))
      return self.__readResponse()
    return

  # --------------------------------------------------------------------------
  def setStepsPerFullRotation(self, motor=0, steps=400):
    if self.__checkMotor(motor):
      cmd = "SETFULLROT " + str(motor) + " " + str(steps)
      self.__sendCommand(cmd)
    return
    
  # --------------------------------------------------------------------------
  def getSubSteps(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETSUBSTEPS " + str(motor))
      return self.__readResponse()
    return

  # --------------------------------------------------------------------------
  def setSubSteps(self, motor=0, substeps=4):
    if self.__checkMotor(motor):
      cmd = "SETSUBSTEPS " + str(motor) + " " + str(substeps)
      self.__sendCommand(cmd)
    return

  # --------------------------------------------------------------------------
  def getWaitTimeBetweenSteps(self, motor=0):
    if self.__checkMotor(motor):
      self.__sendCommand("GETWAITTIME " + str(motor))
      return self.__readResponse()
    return

  # --------------------------------------------------------------------------
  def setWaitTimeBetweenSteps(self, motor=0, waittime=3):
    if self.__checkMotor(motor):
      cmd = "SETWAITTIME " + str(motor) + " " + str(waittime)
      self.__sendCommand(cmd)
    return

  # --------------------------------------------------------------------------
  def setConstAngularVelocity(self, motor=0, direction=CW, time=10.0):
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
























