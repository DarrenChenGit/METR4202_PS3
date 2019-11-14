import cv2
import numpy
import time
from micromelon import *

rc = RoverController()

#rc.connectSerial('/dev/ttyS0')
rc.connectIP('192.168.4.1')
image = Robot.getImageCapture(IMRES.R1280x720)
image = image.astype(numpy.uint8)
cv2.imshow('sad',image)
cv2.waitKey(0)
# all set

