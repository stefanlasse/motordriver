

#
#  This is a sample-program for demonstrating
#  the usage of the motordriver.
#

from Motordriver import *
import time

# get connection to the motordriver
driver = Motordriver(interface = '/dev/ttyUSB0')

QWP = 0   # motor 0 is the one with the quarter wave plate
HWP = 2   # motor 2 is the one with the half wave plate


# --------------------------------------------------------------------------

N = 100

def runTest():
  driver.setWaitTimeBetweenSteps(0, 1)
  driver.moveToAbsolutePosition(0, 0.0, driver.DEGREE)
  while driver.isMoving(0):
    pass
  time.sleep(0.5)
  driver.moveRelative(0, 200, driver.DEGREE)
  time.sleep(1.0)
  driver.stopAllMovements()
  return driver.getPosition(0, driver.DEGREE)

def ADC():
  driver.moveRelative(2, 1, driver.STEPS)
  val = driver.getAnalogValue(2)
  return val
    
  

# --------------------------------------------------------------------------

#acc = 0.0
#driver.moveToAbsolutePosition(0, 10.0, driver.DEGREE)
#driver.moveToAbsolutePosition(2, 50.0, driver.DEGREE)
#while driver.isMoving(0) or driver.isMoving(2):
#  pass
#time.sleep(1)
#driver.goHome()
#while driver.isMoving(0) or driver.isMoving(2):
#  pass

#for i in range(N):
#  acc += runTest()
  
#print acc/N

f = open("data.dat", "w")

for i in range(10000):
  f.write (str(i) + " " + str(ADC()) + "\n")











