from .._rover_controller import RoverController
from .._utils import *
from ..comms_constants import MicromelonType as OPTYPE
from .._binary import bytesToIntArray

_rc = RoverController()

__all__ = [
  'readAccel',
  'readGyro',
  'isFlipped',
  'isRighted',
]

def _div1000(n):
  return n / 1000

def readAccel(n = None):
  """
  Reads the x, y, and z axes of the accelerometer
    Values are float values in units of Gs (9.81m/s^2)
    Returns [x, y, z] iff argument n is None
    Iff n == 0 returns x
    Iff n == 1 returns y
    Iff n == 2 returns z
  """
  if (n != None and (not isNumber(n) or n < 0 or n > 2)):
    raise Exception('Argument to IMU.readAccel must be a number between 0 and 2')
  accel = _rc.readAttribute(OPTYPE.ACCL)
  accel = bytesToIntArray(accel, 2, signed=True)
  accel = list(map(_div1000, accel))
  if (n == None):
    return accel
  return accel[n]

def readGyro(n = None):
  """
  Reads the x, y, and z axes of the gyroscope
    Values are float values in units of degrees per second
    Returns [x, y, z] iff argument n is None
    Iff n == 0 returns x
    Iff n == 1 returns y
    Iff n == 2 returns z
  """
  if (n != None and (not isNumber(n) or n < 0 or n > 2)):
    raise Exception('Argument to IMU.readGyro must be a number between 0 and 2')
  gyro = _rc.readAttribute(OPTYPE.GYRO)
  gyro = bytesToIntArray(gyro, 2, signed=True)
  gyro = list(map(_div1000, gyro))
  if (n == None):
    return gyro
  return gyro[n]

def isFlipped():
  """
  Returns true iff the robot is upside down. False otherwise
  """
  return readAccel()[2] < 0

def isRighted():
  """
  Returns true iff the robot is the right way up. False otherwise
  """
  return readAccel()[2] >= 0
