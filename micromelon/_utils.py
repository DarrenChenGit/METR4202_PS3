from ._logger import Logger, RangeErrorCategory

_logger = Logger()

MAX_SPEED = 30
MAX_DISTANCE = 3276.7
MAX_TIME = 120
MAX_SERVO_DEGREES = 90

def isNumber(x):
  return isinstance(x, (int, float))

# Speed should be in cm/s with a maximum of 30
def restrictSpeed(speed):
  if not isNumber(speed):
    raise Exception('Speed must be a number')

  if abs(speed) > MAX_SPEED:
    # raise Exception('Speed must be between -30 and 30. You gave ' + speed);
    _logger.displayWarning('Speed must be between -{0} and {0}. You gave {1}'.format(MAX_SPEED, speed),
        RangeErrorCategory.SPEED)

  return constrain(speed, -MAX_SPEED, MAX_SPEED)

# Distance given in cm returns errors or distance in mm
def restrictDistance(dist):
  if not isNumber(dist):
    raise Exception('Distance must be a number')

  if abs(dist) > MAX_DISTANCE:
    # raise Exception(
    #     `Distance must be between -${MAX_DISTANCE} and ${MAX_DISTANCE}. You gave ${dist}`);
    _logger.displayWarning('Distance must be between ' +
        '-{0} and {0}. You gave {1}'.format(MAX_DISTANCE, dist), RangeErrorCategory.DISTANCE)
  return constrain(dist, -MAX_DISTANCE, MAX_DISTANCE) * 10

# Returns time in seconds if it's a valid number
def restrictTime(secs):
  if (not isNumber(secs)):
    raise Exception('Seconds must be a number')

  if (secs < 0):
    raise Exception("Seconds can't be negative.  You gave {}".format(secs));

  if (abs(secs) > 120):
    _logger.displayWarning("Command would take {} seconds.  Maximum is {}.".format(secs, MAX_TIME),
        RangeErrorCategory.SECONDS)
  
  return constrain(secs, 0, 120)

# Returns time in seconds if it's a valid number
def restrictRadius(r):
  if (not isNumber(r)):
    raise Exception('Radius must be a number')

  if (r < 0):
    raise Exception('Radius cannot be a negative number. You gave: {}.'.format(r))
  
  return r

def restrictServoDegrees(d):
  if (not isNumber(d)):
    raise Exception('Degrees must be a number')

  if (abs(d) > 90):
    _logger.displayWarning('Degrees must be between ' +
        '-{0} and {0}. You gave {1}'.format(MAX_SERVO_DEGREES, d),
        RangeErrorCategory.SERVO_DEGREES)
  return constrain(d, -MAX_SERVO_DEGREES, MAX_SERVO_DEGREES)


def numberToByteArray(n, byteCount = None):
  hexStr = hex(n)[2:] # skip the '0x'
  if (len(hexStr) % 2 != 0):
    hexStr = '0' + hexStr
  if (not byteCount):
    byteCount = len(hexStr) / 2
  bytesList = []
  for i in range(0, len(hexStr), 2):
    bytesList.append(int(hexStr[i : i + 2], 16))
  
  if (byteCount > len(bytesList)):
    return ([0] * (byteCount - len(bytesList))) + bytesList
  return bytesList

def isBetween(x, low, high):
  """
  Returns true iff x is between low and high inclusive
  Returns false otherwise
  """
  if (not isNumber(x) or not isNumber(low) or not isNumber(high)):
    raise Exception('All arguments to Math.isBetween must be numbers')

  l = low
  h = high
  if (low > high):
    h = low
    l = high

  return (x >= l and x <= h)

def constrain(x, low, high):
  """
  Returns x clamped to between low and high inclusive
  """
  if (not isNumber(x) or not isNumber(low) or not isNumber(high)):
    raise Exception('All arguments to Math.scale must be numbers')
  if (high < low):
    temp = low
    low = high
    high = temp

  if (x >= high):
    return high
  if (x <= low):
    return low
  return x

def scale(x, xmin, xmax, newMin, newMax):
  """
  Returns x scaled from the range xmin - xmax to the range newMin - newMax
  Output is restricted to between newMin and newMax inclusive
  If newMin is greater than newMax or xmin is greater than xmax
  it will be treated and inversely proportional
  """
  if (not isNumber(x) or not isNumber(xmin) or not isNumber(xmax) or
      not isNumber(newMin) or not isNumber(newMax)):
    raise Exception('All arguments to scale must be numbers')
  invert = False
  temp = None
  if xmax < xmin and newMax < newMin:
    temp = xmin
    xmin = xmax
    xmax = temp
    temp = newMin
    newMin = newMax
    newMax = temp
  elif xmax < xmin and newMin < newMax:
    invert = True
    temp = xmin
    xmin = xmax
    xmax = temp
  elif xmin < xmax and newMax < newMin:
    invert = True
    temp = newMin
    newMin = newMax
    newMax = temp
  elif xmin == xmax or newMin == newMax:
    raise Exception('Min and high cannot be the same number in scale function')

  scaled = (((x - xmin) / (xmax - xmin)) * (newMax - newMin)) + newMin
  if invert:
    scaled = newMax - (scaled - newMin)
  if scaled > newMax:
    return newMax
  if scaled < newMin:
    return newMin
  return scaled