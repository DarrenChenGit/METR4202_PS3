from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE, MicromelonImageResolution as IMRES
from .._binary import bytesToAsciiString, stringToBytes, bytesToIntArray
import numpy

_rc = RoverController()

__all__ = [
  'display',
  'setName',
  'getName',
  'getID',
  'getImageCapture',
]

"""
Functions for dealing directly with the robot and not its sensors or actuators
"""

def display(text, label = None):
  """
  Puts the given text or the string representation of it on the robot screen
    If a label is provided the screen will show "`label`: `text`"
    The robot screen can display 15 characters at a time
  """
  text = str(text)
  if label:
    text = str(label) + ': ' + text
  text = text[:15]
  return _rc.writeAttribute(OPTYPE.ROBOT_NAME, [0x7F] + stringToBytes(text))

def setName(name):
  """
  Sets the name of the robot.
  This is cleared with a power cycle and displayed on the robot screen
    Name will be truncated to 11 characters
  """
  name = str(name)[:11]
  return _rc.writeAttribute(OPTYPE.ROBOT_NAME, stringToBytes(name))

def getName():
  """
  Returns the current name of the robot as a string
  """
  name = _rc.readAttribute(OPTYPE.ROBOT_NAME)
  return bytesToAsciiString(name)

def getID():
  """
  Returns the integer id of the robot.
  """
  id = _rc.readAttribute(OPTYPE.BOTID)
  return bytesToIntArray(id, 2, signed=False)[0]

def getImageCapture(resolution):
  """
  For network operation only, will return a raw image of the resolution requested
  """
  if not _rc.isInNetworkMode():
    raise Exception('This operation is only valid over the network to a backpack')
  resVal = resolution
  if (isinstance(resolution, IMRES)):
    resVal = resolution.value
  else:
    resolution = IMRES(resolution)

  if resVal < 1 or resVal > 3:
    raise Exception('Resolution must be 1, 2, or 3 for (640 x 480), (1280 x 720), (1920 x 1088) respectively')
  image = _rc.readAttribute(OPTYPE.RPI_IMAGE, [resVal])
  if resolution == IMRES.R640x480:
    image = numpy.reshape(image, (480, 640, 3))
  elif resolution == IMRES.R1280x720:
    image = numpy.reshape(image, (720, 1280, 3))
  else:
    image = numpy.reshape(image, (1088, 1920, 3))
  return image

