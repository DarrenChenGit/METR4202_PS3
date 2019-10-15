
import numpy
import time
from micromelon import *

rc = RoverController()

#rc.connectSerial('/dev/ttyS0')
rc.connectIP('192.168.4.1')
# all set
print('Robot name: ' + Robot.getName())
Robot.display('Hello')
Motors.moveDistance(15, 7.5);
Motors.turnDegrees(180);
