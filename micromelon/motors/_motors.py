import time
import math
from .._rover_controller import RoverController
from .._utils import *
from ..comms_constants import MicromelonType as OPTYPE
from .._binary import intArrayToBytes

_rc = RoverController()

__all__ = [
  'write',
  'stop',
  'moveDistance',
  'turn',
  'turnDegrees',
  'setDegreesOffset',
]

_TRACK_LENGTH = 8.5 # cm - axle to axle
_ROBOT_WIDTH = 10.5 # cm - middle of track to middle of track
_AXLE_HYPOTHENUSE = math.sqrt(_TRACK_LENGTH * _TRACK_LENGTH + _ROBOT_WIDTH * _ROBOT_WIDTH)
_SLIP_FACTOR = (_AXLE_HYPOTHENUSE / 2 * _TRACK_LENGTH / _ROBOT_WIDTH) * 0.75

_degreesCalibrationOffset = 0

def write(left, right = None, secs = 0):
  """
  Speeds must be between -30 and 30 (cm/s).
  If only one argument is given then both motors will be set to the same speed.
  Seconds is an optional argument, without it the function returns immediately 
    and the motors will stay at the set speed until explicitly changed.
    If it's included then this is a blocking function and 
    the motors will run at the specified speeds for that many seconds and then stop.  
  """
  if right == None:
    right = left
  left = restrictSpeed(left)
  right = restrictSpeed(right)
  secs = restrictTime(secs)

  _rc.writeAttribute(OPTYPE.MOTOR_SET, _buildMotorPacketData([left, right]))
  if secs != 0:
    time.sleep(secs)
    _rc.writeAttribute(OPTYPE.MOTOR_SET, _buildMotorPacketData([0, 0]))

def stop():
  """
  Stops left and right motors
  """
  write(0)
  
def moveDistance(lDist, lSpeed = 15, rDist = None, rSpeed = None, syncStop = False):
  """
  Drives each side of the robot by the distances and speeds specified.
    Distances must be a number of cm.
    Speed is an optional argument and defaults to 15.
    If right values aren't given then they default to the same as the left
    If syncstop is true then both motors will be stopped as soon as one has finished its distance operation
  """
  if (rSpeed == None):
    rSpeed = lSpeed
  lSpeed = restrictSpeed(lSpeed)
  rSpeed = restrictSpeed(rSpeed)

  if (rDist == None):
    rDist = lDist
  lDist = restrictDistance(lDist)
  rDist = restrictDistance(rDist)

  if (lDist == 0 and rDist == 0):
    write(0)
    return

  if (lSpeed < 0):
    lDist *= -1
    lSpeed *= -1

  if (rSpeed < 0):
    rDist *= -1
    rSpeed *= -1
  
  motorValues = [lSpeed, rSpeed, lDist, rDist, syncStop]
  # print('Motor values: ' + str(motorValues))
  _rc.writeAttribute(OPTYPE.MOTOR_SET, _buildMotorPacketData(motorValues))

def turn(speed, secs = None, radius = 0, reverse = False):
  """
  Makes the robot turn at the specified speed for the specified number of seconds.
    If seconds isn't specified it will return immediately and leave the motors turning.
    Speeds are cm/s must be between -30 and 30 inclusive. Negative speed is a left turn.
    By default it turns on the spot, but a radius in cm can be specified.
  """
  speed = restrictSpeed(speed)
  radius = restrictRadius(radius)

  params = _calcMotorSpeedsAndTime(speed, radius, None, reverse)
  speeds = params['speeds']

  if (secs != None):
    secs = restrictTime(secs)
  else:
    secs = 0
  write(speeds[0], speeds[1], secs)

def turnDegrees(degrees, speed = 15, radius = 0, reverse = False):
  """
  Turns the robot by the specified number of degrees.
    Speed must be between -30 and 30 (cm/s) inclusive.
    Negative speed is a left turn.
    If no degrees is specified the robot will turn indefinitely, otherwise it will stop after turning the specified number of degrees.
    By default it turns on the spot, but a radius in cm can be specified.
  """
  speed = restrictSpeed(speed)
  radius = restrictRadius(radius)
  if (not isNumber(degrees)):
    raise Exception('Degrees must be a number')

  if (degrees == 0):
    return True
  
  params = _calcMotorSpeedsAndTime(speed, radius, degrees, reverse)

  # Both the below approaches should work:
  #   Time control prevents error accumulation in the encoders but relies on good latency
  return moveDistance(params['distances'][0], params['speeds'][0],
      params['distances'][1], params['speeds'][1], True)
  # return write(params['speeds'][0], params['speeds'][1], params['seconds'])

def setDegreesOffset(offset):
  """
  Applies this number as a difference to all degrees arguments in the Motor control functions.
  If it is big enough to change the sign of degrees it will not be applied.
  """
  if (not isNumber(offset)):
    raise Exception('Degrees offset must be a number')
  global _degreesCalibrationOffset
  _degreesCalibrationOffset = offset

def _buildMotorPacketData(d):
  if len(d) == 2:
    d = d + [0, 0, 0]
  # Scale speeds to 8 bit

  d[0] = round(d[0] / max(30, abs(d[0])) * 127)
  d[1] = round(d[1] / max(30, abs(d[1])) * 127)

  d[2] = round(d[2])
  d[3] = round(d[3])

  # Allocate lspeed, rspeed, ldist * 2, rdist * 2, syncstop
  data = intArrayToBytes(d[0:2], 1) + intArrayToBytes(d[2:4], 2) + [0]

  if d[4]: # Write 1 or 0 for syncstop
    data[6] = 1
  else:
    data[6] = 0

  return data

# Speed in cm/s and radius in cm, degrees should not be zero
def _calcMotorSpeedsAndTime(speed, radius, degrees = None, reverse = False):
  lDist = None
  rDist = None
  global _degreesCalibrationOffset
  d = degrees
  if (degrees == None):
    d = 90
  else:
    if (degrees < 0):
      degrees += (-1 * _degreesCalibrationOffset)
    else:
      degrees += _degreesCalibrationOffset

    # only apply modified degrees if doesn't cause a sign change
    if (not(d < 0 and degrees > 0) and degrees != 0):
      d = degrees

  if (speed < 0):
    speed *= -1
    d *= -1

  if (d > 0):
    lDist = d * math.pi / 180 * (radius + (_ROBOT_WIDTH / 2) + _SLIP_FACTOR)
    rDist = d * math.pi / 180 * (radius - (_ROBOT_WIDTH / 2) - _SLIP_FACTOR)
  else:
    lDist = (-d) * math.pi / 180 * (radius - (_ROBOT_WIDTH / 2) - _SLIP_FACTOR)
    rDist = (-d) * math.pi / 180 * (radius + (_ROBOT_WIDTH / 2) + _SLIP_FACTOR)

  maxDist = max(abs(lDist), abs(rDist))
  meanDist = (min(abs(lDist), abs(rDist)) + maxDist) / 2

  # Scale so max motor speed will be 30cm/s
  seconds = meanDist / speed
  if (maxDist / seconds > 30):
    seconds = maxDist / 30

  lSpeed = lDist / seconds
  rSpeed = rDist / seconds

  # Speed direction has been set according to sign of distance
  # Distance should always be positive
  lDist = abs(lDist)
  rDist = abs(rDist)

  if reverse:
    lSpeed *= -1
    rSpeed *= -1

  # print('Speeds: ' + str([lSpeed, rSpeed]))
  # print('Distances: ' + str([lDist, rDist]))
  # print('Seconds: ' + str(seconds))
  return {
    'speeds': [lSpeed, rSpeed],
    # Don't include distances or time if no degrees was specified
    'distances': [lDist, rDist] if degrees != None else None,
    'seconds': seconds if degrees != None else None
  }
