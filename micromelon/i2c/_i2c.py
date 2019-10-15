from .._rover_controller import RoverController
from ..comms_constants import MicromelonType as OPTYPE

from .._utils import *

_rc = RoverController()

__all__ = [
  'read',
  'write',
  'scan',
]

def read(address, register, byteCount, addressIs7Bit = True):
  """
  Reads the specified number of bytes from the register of the device
  at the given address.
    If fourth argument is True then address will be treated as 7 bits
    and is shifted right one bit and then the R/W bit is set
    Returns array of bytes read from the address
  """
  if not _rc.isInBluetoothMode():
    raise Exception('Expansion header used by UART.  I2C not available')
  sanitisedAddress = None
  if (addressIs7Bit):
    if (address > 127):
      raise Exception('I2C address is too large to be 7 bit (>127)')
    sanitisedAddress = (address << 1) | 1; # 1 indicates read
  else:
    sanitisedAddress = address | 1
  
  if (register > 0xFF):
    raise Exception('I2C register >255')

  return _rc.readAttribute(OPTYPE.I2C_HEADER, [sanitisedAddress, register, byteCount])

def write(address, register, value, byteCount, addressIs7Bit = True):
  """
  Writes the specified data (value) to the register of the device at the given address.
    If fifth argument is True then address will be treated as 7 bits
    and is shifted right one bit
  """
  if not _rc.isInBluetoothMode():
    raise Exception('Expansion header used by UART.  I2C not available')
  sanitisedAddress = None
  if (addressIs7Bit):
    if (address > 127):
      raise Exception('I2C address is too large to be 7 bit (>127)')
    sanitisedAddress = (address << 1) & 0xFE; # 0 for LSB indicates write
  else:
    sanitisedAddress = address & 0xFE

  if (register > 0xFF):
    raise Exception('I2C register >255')

  bytesToWrite = numberToByteArray(value, byteCount)
  if (byteCount < bytesToWrite.length):
    raise Exception('I2C Data to write requires at least {} bytes'.format(len(bytesToWrite)))

  return _rc.writeAttribute(OPTYPE.I2C_HEADER, [sanitisedAddress, register] + bytesToWrite)
  

def scan():
  """
  Scans for any I2C devices connected to the expansion header
    Returns an array of devices addresses found.  Empty array if none found
  """
  if not _rc.isInBluetoothMode():
    raise Exception('Expansion header used by UART.  I2C not available')
  return _rc.readAttribute(OPTYPE.I2C_HEADER)
