

#
#  Motordriver firmware update script
#

from Motordriver import *
import time
import os


drvInterface = "/dev/ttyUSB0" 

motorList = [0, 1, 2, 3]

# values to be saved and restored
idn = "idn"
optZeroPos =      [0, 0, 0, 0]
gearRatio =       [0.0, 0.0, 0.0, 0.0]
stepsPerFullRot = [0, 0, 0, 0]
subSteps =        [0, 0, 0, 0]
stepMult =        [0.0, 0.0, 0.0, 0.0]
stepUnit =        [0, 0, 0, 0]
waitTime =        [0, 0, 0, 0]
forbZoneStart =   [0, 0, 0, 0]
forbZoneStop =    [0, 0, 0, 0]


# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# saves the current device configuration
# --------------------------------------------------------------------------
def saveConfiguration():
  idn = driver.getIDN()
  for i in motorList:
    optZeroPos[i] = driver.getOpticalZeroPosition(i)
    gearRatio[i] = driver.getGearRatio(i)
    stepsPerFullRot[i] = driver.getStepsPerFullRotation(i)
    subSteps[i] = driver.getSubSteps(i)
    #stepMult[i] = 0
    #stepUnit[i] = 0
    waitTime[i] = driver.getWaitTimeBetweenSteps(i)
    #forbZoneStart[i] = 0
    #forbZoneStop[i] = 0

def restoreConfiguration():
  driver.setIDN(idn)
  for i in motorList:
    driver.setOpticalZeroPosition(i, optZeroPos[i])
    driver.setGearRatio(i, gearRatio[i])
    driver.setStepsPerFullRotation(i, stepsPerFullRot[i])
    driver.setSubSteps(i, subSteps[i])
    #driver.setStepMultiplier...
    #driver.setStepUnit...
    driver.setWaitTimeBetweenSteps(i, waitTime[i])
    #driver.setForbiddenZoneStart...
    #driver.setForbiddenZoneStop...


# ==========================================================================

# get connection to the motordriver
driver = Motordriver(interface = drvInterface)

# save configuration
saveConfiguration()

# close the python interface
del driver

# now update firmware
print "press reset within 1 second"
time.sleep(1)
subp = "avrdude -D -F -p m644 -P " + drvInterface + " -c stk500v1 -b 115200 -U flash:w:motordriver.hex"
update = os.popen(subp)
result = update.read()
print result


# get connection to the motordriver again
driver = Motordriver(interface = drvInterface)

# make a factory reset
driver.factoryReset()

# restore the previous saved configuration
restoreConfiguration()
driver.saveCurrentConfiguration()

del driver

print "Update finished."













