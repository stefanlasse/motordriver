
from Motordriver import *

mot = Motordriver(interface = '/dev/ttyUSB0')
comp = mot.getIDN()

err = 0
for i in range(10):
  resp = mot.getIDN()
  if resp is comp:
    pass
  else:
    err += 1
    
print "error: " + str(err)