

#
#  Factory Test for the motordriver
#

from Motordriver import *
import time

# get connection to the motordriver
driver = Motordriver(interface = '/dev/ttyUSB0')

# contains the connected motors
motorList = [0, 2]
position  = [0, 0, 0, 0]

# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# check IDN
# --------------------------------------------------------------------------
def checkIDN():
  print "Check IDN"
  saveIDN = driver.getIDN()
  driver.setIDN("factorytest")
  idn = driver.getIDN()
  if idn != "factorytest":
    print "ERROR: IDN test failed"
   
  driver.setIDN(saveIDN)
  return


# --------------------------------------------------------------------------
# zero all motors
# --------------------------------------------------------------------------
def checkZeroRun():
  print "Check ZERO RUN"
  for i in motorList:
    driver.zeroMotor(i)
    time.sleep(15)
  
  for i in motorList:
    position[i] = driver.getPosition(i, driver.DEGREE)
    if position[i] > 1e-5:
      print "ERROR: ZERORUN test failed for motor " + str(i)
  return


# --------------------------------------------------------------------------
# check motor movement to absolute position
# --------------------------------------------------------------------------
def checkAbsoluteMovement():
  print "Check ABSOLUTE MOVEMENT"
  newPosition = [0.0, 0.0, 0.0, 0.0]
  for i in motorList:
    position[i] = driver.getPosition(i, driver.DEGREE)
    driver.moveToAbsolutePosition(i, (i+1.0)*10.0, driver.DEGREE)
  
  while driver.isMoving(0) or driver.isMoving(1) or driver.isMoving(2) or driver.isMoving(3):
      pass
  
  for i in motorList:
    newPosition[i] = driver.getPosition(i)
    print newPosition[i]
  
  # check positions
  for i in motorList:
    print "diff: " + str(abs(newPosition[i] - (i+1.0)*10.0))
    if abs(newPosition[i] - (i+1.0)*10.0) > 0.0675:
      print "ERROR: MOVEABS test failed for motor " + str(i)
  return
  

# --------------------------------------------------------------------------
# check motor relative movement
# --------------------------------------------------------------------------
def checkRelativeMovement():
  print "Check RELATIVE MOVEMENT"
  newPosition = [0.0, 0.0, 0.0, 0.0]
  for i in motorList:
    driver.moveRelative(i, -((i+1.0)*10.0), driver.DEGREE)
  
  while driver.isMoving(0) or driver.isMoving(1) or driver.isMoving(2) or driver.isMoving(3):
      pass
  
  for i in motorList:
    newPosition[i] = driver.getPosition(i, driver.DEGREE)
    if newPosition[i] > 0.0675:
      print "ERROR: MOVEREL test failed for motor " + str(i)
  return

# --------------------------------------------------------------------------
# check optical zero position
# --------------------------------------------------------------------------
def checkOpricalZeroPosition():
  print "Check OPTICAL ZERO POSITION"
  return

# --------------------------------------------------------------------------
# check motordriver settings
# --------------------------------------------------------------------------

# gear ratio
def checkGearRatio():
  print "Check GEAR RATIO"
  saveGearRatio = [0.0, 0.0, 0.0, 0.0]
  for i in range(4):
    saveGearRatio[i] = driver.getGearRatio(i)
  
  for i in range(4):
    driver.setGearRatio(i, (i+1.0)*1.123)
  
  for i in range(4):
    gr = driver.getGearRatio(i)
    if abs(gr - (i+1.0)*1.123) > 1e-3:
      print "ERROR: GEAR RATIO test failed for motor " + str(i)
  
  for i in range(4):
    driver.setGearRatio(i, saveGearRatio[i])
  return

# steps per full rotation 
def checkStepsPerFullRotation():
  print "Check STEPS PER FULL ROTATION"
  save = [0, 0, 0, 0]
  for i in range(4):
    save[i] = driver.getStepsPerFullRotation(i)
  
  for i in range(4):
    driver.setStepsPerFullRotation(i, 200)
  
  for i in range(4):
    val = driver.getStepsPerFullRotation(i)
    if val != 200:
      print "ERROR: STEPS PER FULL ROTATION test failed for motor " + str(i)
  
  for i in range(4):
    driver.setStepsPerFullRotation(i, save[i])
  return

# sub steps
def checkSubSteps():
  print "Check SUB STEPS"
  save = [0, 0, 0, 0]
  for i in range(4):
    save[i] = driver.getSubSteps(i)
  
  for i in range(4):
    driver.setSubSteps(i, 2**(i+1))
  
  for i in range(4):
    val = driver.getSubSteps(i)
    if val != 2**(i+1):
      print "ERROR: SUB STEPS test failed for motor " + str(i)
  
  for i in range(4):
    driver.setSubSteps(i, save[i])
  return

# wait time between steps
def checkWaitTimeBetweenSteps():
  print "Check WAIT TIME BETWEEN STEPS"
  save = [0, 0, 0, 0]
  for i in range(4):
    save[i] = driver.getWaitTimeBetweenSteps(i)
  
  for i in range(4):
    driver.setWaitTimeBetweenSteps(i, (i+1)**2)
  
  for i in range(4):
    val = driver.getWaitTimeBetweenSteps(i)
    if val != (i+1)**2:
      print "ERROR: WAIT TIME test failed for motor " + str(i)
  
  for i in range(4):
    driver.setWaitTimeBetweenSteps(i, save[i])
  return

# --------------------------------------------------------------------------
# check constant angular velocity
# --------------------------------------------------------------------------
def checkConstAngularVelocity():
  print "Check CONSTANT ANGULAR VELOCITY"
  for i in motorList:
    driver.setConstAngularVelocity(i, driver.CW)

  for i in motorList:
    val = driver.isMoving(i)
    if val != True:
      print "ERROR: CONST ANGULAR VELOCITY test failed for motor " + str(i)
  
  driver.stopAllMovements()
  
  for i in motorList:
    val = driver.isMoving(i)
    if val:
      print "ERROR: STOP ALL MOTOT MOVEMENTS test failed for motor " + str(i)
  return







# ==========================================================================
# factory check
# ==========================================================================

checkIDN()
checkZeroRun()
checkAbsoluteMovement()
checkRelativeMovement()
checkOpricalZeroPosition()
checkGearRatio()
checkStepsPerFullRotation()
checkSubSteps()
checkWaitTimeBetweenSteps()













