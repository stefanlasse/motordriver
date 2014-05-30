

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

def measureStokesParameter():
  # first move the motors to zero position
  driver.moveToAbsolutePosition(QWP, 0, driver.DEGREE)
  driver.moveToAbsolutePosition(HWP, 0, driver.DEGREE)
  while driver.isMoving(QWP) or driver.isMoving(HWP):
    # wait until both of them reached their
    # zero positions
    pass

  time.sleep(3) # give some time for power meter to settle
  # measure power here: S0 = P(QWP) + P(HWP)
  # measure power here: S1 = P(QWP) - P(HWP)
  print "got S0 and S1"
  
  # now move QWP to 45 degree and HWP to 22.5 degree
  driver.moveToAbsolutePosition(QWP, 45, driver.DEGREE)
  driver.moveToAbsolutePosition(HWP, 22.5, driver.DEGREE)
  while driver.isMoving(QWP) or driver.isMoving(HWP):
    # wait until both of them reached their
    # zero positions
    pass
  
  time.sleep(3) # give some time for power meter to settle
  # measure power here: S2 = P(QWP) - P(HWP)
  print "got S2"
  
  # now let QWP at 45 degree and move HWP to 45 degree
  driver.moveToAbsolutePosition(HWP, 45, driver.DEGREE)
  while driver.isMoving(QWP) or driver.isMoving(HWP):
    # wait until both of them reached their
    # zero positions
    pass
  
  time.sleep(3) # give some time for power meter to settle
  # measure power here: S3 = P(QWP) - P(HWP)
  print "got S3"
  return

def getTheIDfromDevice():
  return driver.getIDN()

for i in range(1,10000001):
  print "Meas " + str(i) + " " + getTheIDfromDevice()
  #driver.setWaitTimeBetweenSteps(i % 4, i % 20);
  #measureStokesParameter()
  time.sleep(0.2)
  print " "


















