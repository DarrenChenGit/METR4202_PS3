from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE
from .._binary import bytesToIntArray

_rc = RoverController()

__all__ = [
  'readAll',
  'readLeft',
  'readRight',
]

def readAll():
  """
  Returns distances in cm as floats [left, right]
  """
  result = _rc.readAttribute(OPTYPE.TIME_OF_FLIGHT)
  mm = bytesToIntArray(result, 2, signed=False)
  return [mm[0] / 10, mm[1] / 10]

def readLeft():
  """
  Returns distance in cm as a float from left sensor
  """
  return readAll()[0]

def readRight():
  """
  Returns distance in cm as a float from rightt sensor
  """
  return readAll()[1]
