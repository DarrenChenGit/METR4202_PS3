from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE

_rc = RoverController()

__all__ = [
  'readVoltage',
  'readPercentage',
]

def readVoltage():
  """
  Returns float value of the battery voltage
  """
  return _rc.readAttribute(OPTYPE.BATTERY_VOLTAGE)[0]

def readPercentage():
  """
  Returns integer percetage of battery charge
  """
  return _rc.readAttribute(OPTYPE.STATE_OF_CHARGE)[0]
