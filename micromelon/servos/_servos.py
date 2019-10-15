from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE

from .._utils import *

_rc = RoverController()

__all__ = [
  'left',
  'right',
  'set',
  'read',
]

def _setServos(s1, s2):
  """
  Sets both servos
  """
  # Flag to leave a servo as it is is 0xFF (255)
  # so only apply the 90 offset if it's in a range to be set
  if s1 <= 90:
    s1 += 90
  if s2 <=90:
    s2 += 90
  return _rc.writeAttribute(OPTYPE.SERVO_MOTORS, [s1, s2])

  """
  Functions to control servo motors connected to the three pin headers
  on the rear of the robot
  """

def left(degrees):
  """
  Turns the left servo to the specified number of degrees
    degrees must be between -90 and 90
  """
  degrees = restrictServoDegrees(degrees)
  return _setServos(degrees, 0xFF)

def right(degrees):
  """
  Turns the right servo to the specified number of degrees
    degrees must be between -90 and 90
  """
  degrees = restrictServoDegrees(degrees)
  return _setServos(0xFF, degrees)

def set(s1, s2):
  """
  Sets degrees of both servos
    left to s1
    right to s2
    degrees must be between -90 and 90
  """
  s1 = restrictServoDegrees(s1)
  s2 = restrictServoDegrees(s2)
  return _setServos(s1, s2)

def read():
  """
  Returns the current set point of the left and right servos in degrees
  """
  degrees = _rc.readAttribute(OPTYPE.SERVO_MOTORS)
  # unsigned read so offset back to -90 to 90 range
  return [degrees[0] - 90, degrees[1] - 90]
