import math
import random
from enum import Enum

from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE
from .._binary import bytesToIntArray

_rc = RoverController()

__all__ = [
  'CS',
  'random',
  'randomHue',
  'rgb',
  'pick',
  'hsv',
  'hue',
  'blend',
  'readAllSensors',
  'readSensor',
  'sensorSees',
  'rgbToHsv',
  'hsvToRgb',
  'hexToRgb',
  'rgbToHex',
]

class CS(Enum):
  """
  Enum of attributes that can be read from the colour sensor
    Values correspond to attribute position in the sensor read array
  """
  HUE = 0
  RED = 1
  GREEN = 2
  BLUE = 3
  BRIGHT = 4
  ALL = 5

def random():
  """
  Returns a random colour in the form [r, g, b]
  """
  r = random.randint(0, 255)
  g = random.randint(0, 255)
  b = random.randint(0, 255)
  return [r, g, b]

def randomHue():
  """
  Returns a random hue colour with full saturation and value in the form [r, g, b]
  """
  return hsvToRgb(random.randint(0, 359), 1, 1)

def rgb(r, g, b):
  """
  Returns [r, g, b] if the arguments are in a valid range
    Valid range is 0 - 255 inclusive for r, g, and b
  """
  return _checkRGB(r, g, b)

def pick(r, g, b):
  """
  Alias for rgb
  """
  return rgb(r, g, b)

def hsv(h, s, v):
  """
  Returns the [r, g, b] representation of the hsv colour if arguments are in the valid ranges
    Valid range is 0 - 359 inclusive for hue and 0 - 1 inclusive for saturation and value
  """
  c = _checkHSV(h, s, v)
  return hsvToRgb(c[0], c[1], c[2])

def hue(hue):
  """
  Returns the [r, g, b] for the given hue with 1 for both saturation and value
    hue must be between 0 and 359 inclusive
  """
  return hsv(hue, 1, 1)

def blend(c1, c2, ratio, isHSV = False):
  """
  Returns [r, g, b] of c1 and c2 combined with the given ratio
    c1 and c2 should be either [r, g, b] or [h, s, v] values
    If using hsv then set isHSV to True
  """
  if (ratio > 1 or ratio < 0):
    raise Exception('Blend ratio must be between 0 and 1')
  if isHSV:
    c1 = hsv(c1[0], c1[1], c1[2])
    c2 = hsv(c2[0], c2[1], c2[2])

  checked = _checkRGB(c1[0], c1[1], c1[2])
  if (checked[0] == False):
    return checked

  checked = _checkRGB(c2[0], c2[1], c2[2])
  if (checked[0] == False):
    return checked

  r = round(c1[0] * (1 - ratio) + c2[0] * ratio)
  g = round(c1[1] * (1 - ratio) + c2[1] * ratio)
  b = round(c1[2] * (1 - ratio) + c2[2] * ratio)
  return [r, g, b]

def readAllSensors(option = CS.ALL):
  """
  Reads the three colour sensor values.
    You'll need to calibrate with these values
    Options are CS.HUE, CS.RED, CS.GREEN, CS.BLUE, CS.BRIGHT, and CS.ALL.
    Returns an array of results [left, middle, right]
    If CS.ALL is chosen, then each of left, middle, and right will be an array of the 5 other options in order.
    Otherwise it will be a single number for each sensor.
  """
  reading = _readRawColourFromRobot()
  if (not reading):
    raise Exception('Colour sensor read failed')

  if (option.value < 0 or option.value > 5):
    raise Exception('Invalid Colour Sensor read option')
  
  if (option == CS.ALL):
    return reading

  option = option.value
  return [reading[0][option], reading[1][option], reading[2][option]]

def readSensor(option = CS.HUE, sensor = 1):
  """
  Reads a value from one of the colour sensors.
    You'll need to calibrate for this value.
    Options are CS.HUE, CS.RED, CS.GREEN, CS.BLUE, CS.BRIGHT, and CS.ALL.
    Sensor argument can be 0, 1, or 2 for left, middle, and right
    If CS.ALL is chosen, then return value will be an array of [hue, r, g, b, w]
    Otherwise a single number will be returned
  """
  if (sensor < 0 or sensor > 2):
    raise Exception('Argument for sensor must be 0, 1, or 2')
  
  reading = readAllSensors(option)
  return reading[sensor]

def sensorSees(option, sensor = None):
  """
  Returns True iff a one of r, g, or b is bresent and greater than the other two
    For brightness it returns true iff r, g, and b all return similar values and the W value is greater than 100
  """
  if (sensor != None and (sensor < 0 or sensor > 2)):
    raise Exception('Argument for sensor must be 0, 1, or 2')

  if (option < 1 or option > 4):
    raise Exception('Invalid sensorSees colour option')

  def colourIs(readValue, option):
    WHITE_DEVIATION_THRESHOLD = 30
    BRIGHTNESS_THRESHOLD = 100
    MINIMUM_COLOUR_VALUE = 30
    if (option == CS.BRIGHT):
      average = (readValue[CS.RED] + readValue[CS.GREEN] + readValue[CS.BLUE]) / 3
    
      if (readValue[CS.BRIGHT] > BRIGHTNESS_THRESHOLD):
        if (abs(readValue[CS.RED] - average) < WHITE_DEVIATION_THRESHOLD and
            abs(readValue[CS.GREEN] - average) < WHITE_DEVIATION_THRESHOLD and
            abs(readValue[CS.BLUE] - average) < WHITE_DEVIATION_THRESHOLD):
          return True
      return False
    if (option == CS.RED):
      return (readValue[CS.RED] > readValue[CS.BLUE] and
              readValue[CS.RED] > readValue[CS.GREEN] and
              readValue[CS.RED] > MINIMUM_COLOUR_VALUE)

    if (option == CS.GREEN):
      return (readValue[CS.GREEN] > readValue[CS.RED] and
              readValue[CS.GREEN] > readValue[CS.BLUE] and
              readValue[CS.GREEN] > MINIMUM_COLOUR_VALUE)

    if (option == CS.BLUE):
      return (readValue[CS.BLUE] > readValue[CS.RED] and
              readValue[CS.BLUE] > readValue[CS.GREEN] and
              readValue[CS.BLUE] > MINIMUM_COLOUR_VALUE)
    return False
  
  reading = readAllSensors()
  if (sensor == None):
    return (colourIs(reading[0], option) and
          colourIs(reading[1], option) and
          colourIs(reading[2], option))
  return colourIs(reading[sensor], option)

def rgbToHsv(r, g, b):
  """
  Converts an RGB color value to HSV. Conversion formula
  adapted from http://en.wikipedia.org/wiki/HSV_color_space.
  Assumes r, g, and b are contained in the set [0, 255] and
  returns h in the set [0, 360], s, and v in the set [0, 1].
  Returns [h, s, v]
  """
  r /= 255.0
  g /= 255.0
  b /= 255.0

  mx = max(r, g, b)
  mn = min(r, g, b)
  v = mx
  d = mx - mn
  s = 0 if mx == 0 else d / mx
  h = None

  if (mx == mn):
    h = 0; # achromatic
    return [h, s, v]
  
  if r == mx:
    h = (60 * ((g-b)/d) + 360) % 360
  if g == mx:
    h = (60 * ((b-r)/d) + 120) % 360
  if b == mx:
    h = (60 * ((r-g)/d) + 240) % 360

  return [round(h) % 360, s, v]


def hsvToRgb(h, s, v):
  """
  Converts an HSV color value to RGB. Conversion formula
  adapted from http://en.wikipedia.org/wiki/HSV_color_space.
  Assumes h is [0, 360], and s and v are contained in the set [0, 1]
  returns r, g, and b in the set [0, 255].
  Returns [r, g, b]
  """
  r = None
  g = None
  b = None

  h60 = h / 60.0
  h60f = math.floor(h60)
  hi = int(h60f) % 6
  f = h60 - h60f
  p = v * (1 - s)
  q = v * (1 - f * s)
  t = v * (1 - (1 - f) * s)
  r, g, b = 0, 0, 0
  if hi == 0: r, g, b = v, t, p
  elif hi == 1: r, g, b = q, v, p
  elif hi == 2: r, g, b = p, v, t
  elif hi == 3: r, g, b = p, q, v
  elif hi == 4: r, g, b = t, p, v
  elif hi == 5: r, g, b = v, p, q
  r, g, b = round(r * 255), round(g * 255), round(b * 255)
  return [r, g, b]

def hexToRgb(hex):
  """
  Converts hex colour codes eg. #FFF or #00FF0F to rgb array
    Returns [r, g, b] each in the range of 0 - 255 inclusive
  """
  # strip '#'
  if (hex[0] == '#'):
    hex = hex[1:]
  if len(hex) == 3:
    # Expand shorthand form (e.g. "03F") to full form (e.g. "0033FF")
    return [int(hex[i] * 2, 16) for i in (0, 1, 2)]
  return [int(hex[i:i+2], 16) for i in (0, 2, 4)]


def rgbToHex(r, g, b):
  """
  Converts r, g, b colour tohex colour codes eg. #FFF or #00FF0F
    Returns hexadecimal string beginning with '#' eg. #00FF0F
  """
  return "#{0:02x}{1:02x}{2:02x}".format(r, g, b)

###############################################################################
###############################################################################
# Helper functions
###############################################################################
###############################################################################

def _parseColourArg(c):
  if not c:
    return None
  rgb = None
  if (type(c) is str):
    rgb = hexToRgb(c)
    if rgb:
      return rgb
  elif (type(c) is list):
    if (len(c) == 3 and _checkRGB(c[0], c[1], c[2])):
      return c
      # return c.map(x => Math.min(Math.max(0, x), 255));
  return None

# Convert colour sensor reading into array of three [h, r, g, b, w] readings
def _readRawColourFromRobot():
  raw = _rc.readAttribute(OPTYPE.COLOUR_ALL)
  raw = bytesToIntArray(raw, 2, signed=False)
  if raw == None:
    return None
  parsed = []
  if (len(raw) == 3):
    parsed.append([raw[0]] + hsvToRgb(raw[0], 1, 1) + [128])
    parsed.append([raw[1]] + hsvToRgb(raw[1], 1, 1) + [128])
    parsed.append([raw[2]] + hsvToRgb(raw[2], 1, 1) + [128])
    return parsed
    
  for i in range(len(raw)):
    # Scaling for sensor on 10 integration cycles and max count of 1024
    raw[i] = (raw[i] / 10240) * 255
    raw[i] = round(raw[i] * 100) / 100

  parsed.append([rgbToHsv(raw[0], raw[1], raw[2])[0]] + raw[:4])
  parsed.append([rgbToHsv(raw[4], raw[5], raw[6])[0]] + raw[4:8])
  parsed.append([rgbToHsv(raw[8], raw[9], raw[10])[0]] + raw[8:])
  return parsed

def _checkRGB(r, g, b):
  if (r < 0 or g < 0 or b < 0 or r > 255 or g > 255 or b > 255):
    raise Exception('Invalid RGB colour: r, g, and b should all be between 0 and 255')
  return [r, g, b]

def _checkHSV(h, s, v):
  if (h < 0 or s < 0 or v < 0 or h > 360 or s > 1 or v > 1):
    raise Exception('Invalid HSV colour: Hue must be between 0 and 360\n' +
      '\t\t s and v must be between 0 and 1')
  return [h, s, v]