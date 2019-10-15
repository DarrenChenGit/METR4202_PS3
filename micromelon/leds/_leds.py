from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE
from ..colour._colour import _parseColourArg

_rc = RoverController()

__all__ = [
  'write',
  'writeAll',
  'off',
]

"""
Functions for controlling the RGB LEDs on the robot
  Top and bottom LEDs are coupled so you can only control 4
  Bottom LEDs mirror the setting of top LED
"""

def write(id, c):
  """
  Sets the colour of a specific LED
    Colour (c) must be in the form [r, g, b] with values between 0 and 255 inclusive
    id must be 1, 2, 3, or 4
  """
  if (id < 1 or id > 4):
    raise Exception('LED id must be 1, 2, 3 or 4. You gave ' + id)

  rgb = _parseColourArg(c)
  if rgb:
    ledArray = [(1 << (id - 1))] + [0, 0, 0] * 4
    offset = ((id - 1) * 3) + 1
    for i in (0, 1, 2):
      ledArray[i + offset] = rgb[i]
    _rc.writeAttribute(OPTYPE.RGBS, ledArray)
    return
  else:
    raise Exception('Invalid colour: ' + c + '.  Note rgb values should be in range 0-255')

def writeAll(c1, c2 = None, c3 = None, c4 = None):
  """
  Set the colour of all LEDs at once
    Arguments must be in the form [r, g, b] with values between 0 and 255 inclusive
    If only one arument is given, all LEDs will be set to that colour
  """
  rgb1 = _parseColourArg(c1)

  if rgb1:
    ledArray = None
    if (c2 and c3 and c4):
      rgb2 = _parseColourArg(c2)
      rgb3 = _parseColourArg(c3)
      rgb4 = _parseColourArg(c4)
      ledArray = rgb1 + rgb2 + rgb3 + rgb4
    elif (c2 or c3 or c4):
      raise Exception('Incorrect number of colours provided.  Should be 1 or 4')
    else:
      ledArray = rgb1 * 4

    return _rc.writeAttribute(OPTYPE.RGBS, [0x0F] + ledArray)

  raise Exception('Invalid Colour - Should be in the form [r, g, b]')


def off():
  """
  Turns all LEDs off (sets colour to black [0, 0, 0])
  """
  _rc.writeAttribute(OPTYPE.RGBS, [0x0F] + [0] * 12)
