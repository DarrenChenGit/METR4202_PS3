import sys
from micromelon import *

rc = RoverController()

try:
  rc.connectSerial()
  Robot.display(sys.argv[1])
except Exception as e:
  print(e)

