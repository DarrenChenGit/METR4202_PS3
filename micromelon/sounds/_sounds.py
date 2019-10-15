import math
import time
from enum import Enum
from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE

from .._utils import *

_rc = RoverController()

__all__ = [
  'playNote',
  'play',
  'off',
]

"""
Functions to control the buzzer on the robot
"""

def playNote(freq, secs = None):
  """
  Sets the buzzer to the given frequency in Hz
    If secs argument is provided, the function will block until that time has elapsed
  """
  # Allow use of NOTES enum as frequency arguments
  if isinstance(freq, Enum):
    freq = freq.value

  if (not isNumber(freq) or freq < 0):
    raise Exception('Note frequency must be a positive number')

  if (secs):
    secs = restrictTime(secs)

  # TODO: Is ceil really the right thing to do here?
  freq = math.ceil(freq)
  littleEndianFreq = [round(freq) & 0xFF, (round(freq) >> 8) & 0xFF]
  result = _rc.writeAttribute(OPTYPE.BUZZER_FREQ, littleEndianFreq)
  if (secs):
    time.sleep(secs)
  return result

def play(id):
  """
  Starts playing a preset tune on the robot
    see what you can find
    non blocking
  """
  # Allow use of TUNES enum elements as arguments
  if isinstance(id, Enum):
    id = id.value
  return _rc.writeAttribute(OPTYPE.BUZZER_TUNE, [id])

def off():
  """
  Turns the buzzer off
    (sets frequency to zero Hz)
  """
  return _rc.writeAttribute(OPTYPE.BUZZER_FREQ, [0])
