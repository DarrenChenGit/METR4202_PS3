import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from micromelon import *

camera = PiCamera()
res = (640, 480) # (1280, 720) (1920, 1088)
camera.resolution = res
rawCapture = PiRGBArray(camera)
print('Starting')
# Allow camera warmup
time.sleep(1)
rc = RoverController()
rc.connectSerial()

smolRes = (5, 3)

while True:
  #print('Loop start')
  rawCapture.truncate(0)
  camera.capture(rawCapture, format="bgr")
  image = rawCapture.array
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  smol = cv2.resize(gray, smolRes, interpolation=cv2.INTER_LINEAR)
  verticals = [0] * smolRes[0]
  # Calc average brightness per column in smolres
  for i in range(smolRes[0]):
    avg = 0
    for j in range(smolRes[1]):
      avg += smol[j][i]
    verticals[i] = avg / smolRes[1]
  # Find the brightest column
  dir = verticals.index(max(verticals))
  # Normalise brightest column around zero
  dir = dir - math.floor(smolRes[0] / 2)
  print('Direction: {}'.format(dir))
  Motors.write((15 - dir * 6) / 2, (15 + dir * 6) / 2)

  #############################################################################
  # not relevant for headless pi
  #cv2.imshow('Gray', gray)
  #cv2.imshow('Smol', cv2.resize(smol, (640, 480), interpolation=cv2.INTER_LINEAR))
  #cv2.waitKey(300)
  #############################################################################
