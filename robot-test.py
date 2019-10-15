import time
from micromelon import *

rc = RoverController()

rc.connectSerial('/dev/ttyS0')
Motors.write(0)
print('Robot name: ' + Robot.getName())
Robot.display('Hello')
Sounds.play(TUNES.UP)
for i in range(0, 360, 50):
  c = Colour.hue(i)
  LEDs.writeAll(c)
  #Sounds.playNote((i+1) * 100)
  print('Ultra: ' + str(Ultrasonic.read()))
  print('IR: ' + str(IR.readAll()))
  print('Accel: ' + str(IMU.readAccel()))
  print('Gyro: ' + str(IMU.readGyro()))
  print('Is flipped: ' + str(IMU.isFlipped()))
  print('Colour: ' + str(Colour.readAllSensors()))
  time.sleep(1)
rc.disarm()

