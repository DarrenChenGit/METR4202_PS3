from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE
from .._binary import bytesToIntArray

_rc = RoverController()

__all__ = [
  'read',
]

def read():
  """
  Returns the number of cm to the nearest object in the Ultrasonic sensor's field of view
  """
  reading = _rc.readAttribute(OPTYPE.ULTRASONIC)
  return bytesToIntArray(reading, 2, signed=False)[0]
