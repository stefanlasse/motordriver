

#
#  This is a demo-program for demonstrating
#  the usage of the motordriver.
#

from Motordriver import *
import time

# get connection to the motordriver
driver = Motordriver(interface = 'COM6')

mot0 = 0   # motor 0 M101
mot1 = 1   # motor 1 M101
mot2 = 2   # motor 2 M102 with ND filter
mot3 = 3   # motor 3 M102 with filter wheel

# contains the connected motors
motorList = [0, 1, 3]


# --------------------------------------------------------------------------

def getValues():
  print "IDN is " + str(driver.getIDN())
  for i in range(4):
    print "GearRatio of motor " + str(i) + " is "+ str(driver.getGearRatio(i))
  for i in range(4):
    print "Steps per full Rotation of motor " + str(i) + " are "+ str(driver.getStepsPerFullRotation(i))
  for i in range(4):
    print "SubSteps of motor " + str(i) + " are "+ str(driver.getSubSteps(i))
  for i in range(4):
    print "ZeroOffset of motor " + str(i) + " is "+ str(driver.getZeroPosition(i)) + " st"
  for i in (motorList):
    print "Current for motor " + str(i) + " is "+ str(driver.getMotorCurrent(i)) + " A"
  for i in range(4):
    print "WaitTime of motor " + str(i) + " is "+ str(driver.getWaitTimeBetweenSteps(i)) + " ms"
  for i in range(4):
    print "Decay of motor " + str(i) + " is "+ str(driver.getMotorDecay(i))
    
  return
  
  
def readValues(number=0):
  tmp = 0.0
  for i in range(number):
    print "run " + str(i) 
    idn = driver.getIDN()
    for i in range(4):
      tmp = driver.getGearRatio(i)
      tmp = driver.getStepsPerFullRotation(i)
      tmp = driver.getSubSteps(i)
      tmp = driver.getZeroPosition(i)
      tmp = driver.getMotorCurrent(i)
      tmp = driver.getWaitTimeBetweenSteps(i)
      tmp = driver.getMotorDecay(i)
    
  print "run " + str(number) + " times"   
  return

  
  
def setValues():
  driver.setGearRatio(0, 3.0)
  driver.setGearRatio(1, 3.0)
  driver.setGearRatio(2, 1.0)
  driver.setGearRatio(3, 1.0)
  return


def ZeroRun():
  print "ZERO RUN"
  for i in motorList:
    driver.zeroMotor(i)
    time.sleep(15)
  return
  

def demoProgram_1():
  #move motor 0 to 45 deg
  driver.moveToAbsolutePosition(mot0, 45.0, driver.DEGREE)
  #move motor 1 to pi/2
  driver.moveToAbsolutePosition(mot1, 0.5, driver.PI)
  while driver.isMoving(mot0) or driver.isMoving(mot1):
    pass
  time.sleep(0.5)
  #move motor 0 to 90 deg
  driver.moveToAbsolutePosition(mot0, 90.0, driver.DEGREE)
  #move motor 1 to pi
  driver.moveToAbsolutePosition(mot1, 1, driver.PI)
  while driver.isMoving(mot0) or driver.isMoving(mot1):
    pass
  time.sleep(0.5)
  #move motor 0 and motor 1 to 0 deg
  driver.moveToAbsolutePosition(mot0, 0.0, driver.DEGREE)
  driver.moveToAbsolutePosition(mot1, 0.0, driver.DEGREE)
  while driver.isMoving(mot0) or driver.isMoving(mot1):
    pass
  time.sleep(0.5)
  return
  
  
def demoProgram_2():
  #move motor 3 to 22.5 deg
  driver.moveToAbsolutePosition(mot3, 22.5, driver.DEGREE)
  while driver.isMoving(mot3):
    pass
  time.sleep(0.5)
  #move motor 3 to 45 deg
  driver.moveToAbsolutePosition(mot3, 45.0, driver.DEGREE)
  while driver.isMoving(mot3):
    pass
  time.sleep(0.5)
  #move motor 3 to 90 deg
  driver.moveToAbsolutePosition(mot3, 90.0, driver.DEGREE)
  while driver.isMoving(mot3):
    pass
  time.sleep(0.5)
  #move motor 3 to 0 deg
  driver.moveToAbsolutePosition(mot3, 0.0, driver.DEGREE)
  while driver.isMoving(mot3):
    pass
  time.sleep(0.5)
  return
  
  
def demoProgram_3():
  #move motor 3 to 0 deg
  driver.moveToAbsolutePosition(mot3, 0.0, driver.DEGREE)
  while driver.isMoving(mot3):
    pass
  time.sleep(1)
  for i in range(3):
    #move motor 3 +60 deg
    driver.moveRelative(mot3, 60.0, driver.DEGREE)
    while driver.isMoving(mot3):
      pass
    time.sleep(0.5)
  return


def setStokesProgramToDriver():
  driver.defineInternalProgramStep(0, [0.0, 0.0, 0.0, 0.0], driver.DEGREE, driver.ABSOLUTE)
  driver.defineInternalProgramStep(1, [45.0, 22.5, 0.0, 0.0], driver.DEGREE, driver.ABSOLUTE)
  driver.defineInternalProgramStep(2, [45.0, 45.0, 0.0, 0.0], driver.DEGREE, driver.ABSOLUTE)
  
  
def setFilterProgramToDriver():
  driver.defineInternalProgramStep(0, [0.0, 0.0, 0.0,   0.0], driver.DEGREE, driver.ABSOLUTE)
  driver.defineInternalProgramStep(1, [0.0, 0.0, 0.0,  60.0], driver.DEGREE, driver.ABSOLUTE)
  driver.defineInternalProgramStep(2, [0.0, 0.0, 0.0, 120.0], driver.DEGREE, driver.ABSOLUTE)
  driver.defineInternalProgramStep(3, [0.0, 0.0, 0.0, 180.0], driver.DEGREE, driver.ABSOLUTE)
  driver.defineInternalProgramStep(4, [0.0, 0.0, 0.0, 240.0], driver.DEGREE, driver.ABSOLUTE)
  driver.defineInternalProgramStep(5, [0.0, 0.0, 0.0, 300.0], driver.DEGREE, driver.ABSOLUTE)


def idnTest(number=0):
  errornumber = 0
  for i in range(number):
    #time.sleep(0.01)
    #idn = driver.getPosition(0, driver.DEGREE)
    idn = driver.getIDN()
    if idn != "test":
      print "IDN test failed at " + str(i)
      print "read IDN is " + str(idn)
      errornumber += 1
    
  print "run " + str(number) + " times"   
  print "with " + str(errornumber)+ " errors"
  return

def OnOffTest(number=0):
  for i in range(number):
    driver.turnOnMotor(0)
    time.sleep(0.1)
    driver.turnOffMotor(0)
    time.sleep(0.1)
    
  print "run " + str(number) + " times"   
  return

# --------------------------------------------------------------------------

getValues()

#readValues(100)

#setStokesProgramToDriver()
#setFilterProgramToDriver()


#for j in range(10):
#  print "Cycle " + str(j) 
#  ZeroRun()
#  for i in range(15):
#    print "Meas " + str(i)  
#    demoProgram_1()
    #demoProgram_2()
#    demoProgram_3()
    #demoProgram_1()


#idnTest(100)    


#OnOffTest(100)  









