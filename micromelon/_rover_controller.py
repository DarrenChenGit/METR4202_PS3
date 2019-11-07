import serial
import time
import math
import signal
import sys
from ._singleton import Singleton
from .comms_constants import MicromelonOpCode as OPCODE, MicromelonType as OPTYPE, MicromelonImageResolution as IMRES
from ._tcp_controller import SerialTCPConnection

class RoverController(metaclass=Singleton):
  """
  Manages connection and packet level communication with the robot
    Is a singleton - get a reference to the instance with constructor

    standard usage
    rc = RoverController()
    rc.connectSerial()
  """
  def __init__(self, port = None, bluetooth = False):
    self._READ_PACKET_TIMEOUT_SECS = 6.0
    self._BLOCKING_READ_TIMEOUT_SECS = 2.0
    signal.signal(signal.SIGINT, _sigint_handler)
    if bluetooth:
      self._connection = None
    elif port:
      self.connectSerial(port)
    else:
      self._connection = None

  def isInBluetoothMode(self):
    return False # BLE python not implemented
  
  def isInNetworkMode(self):
    return isinstance(self._connection, SerialTCPConnection)

  def setReadTimeout(self, seconds = None):
    """
    Timeout used for blocking reads of packets
    """
    if seconds == None:
      self._BLOCKING_READ_TIMEOUT_SECS = 120.0
    else:
      self._BLOCKING_READ_TIMEOUT_SECS = seconds

  def connectSerial(self, newPort = "/dev/ttyS0"):
    """
    Sets the communcation to use serial
      default port is the gpio serial on a raspberry pi
    """
    if isinstance(self._connection, serial.Serial):
      self._connection.close()
      self._connection.port = newPort
      self._connection.open()
    else:
      if self._connection:
        self._connection.close()
      self._connection = serial.Serial(newPort, baudrate=115200, timeout=self._BLOCKING_READ_TIMEOUT_SECS)
    self._connection.flushInput()
    self._connection.flushOutput()
    self.arm()

  def connectIP(self, address = '192.168.4.1', port = 4202):
    if self._connection:
      self._connection.close()
    self._connection = SerialTCPConnection(address, port, self._BLOCKING_READ_TIMEOUT_SECS)
    self._connection.open()
    self._connection.flushInput()
    self._connection.flushOutput()
    try:
      self.arm()
    except Exception as e:
      print('Arming failed, maybe no robot connected')
      print('Using camera only mode')


  def stopRobot(self, waitForAck = False):
    self.writePacket(OPCODE.WRITE, OPTYPE.MOTOR_SET, [0] * 7, waitForAck)
    self.writePacket(OPCODE.WRITE, OPTYPE.BUZZER_FREQ, [0], waitForAck)

  def writeAttribute(self, opType, data):
    """
    Writes an attribute and returns once the ACK packet is received from the robot
    """
    self.writePacket(OPCODE.WRITE, opType, data)

  def readAttribute(self, opType, data = [], timeout = None):
    """
    Blocking read - returns the raw data from robot response
    """
    self.writePacket(OPCODE.READ, opType, data, False)
    p = self.waitForPacket(OPCODE.ACK, opType, timeout)
    return p[3:] # Only return the data

  def writePacket(self, opCode, opType, data = [], waitForAck = True):
    """
    Writes the packet over transport
    Blocks and waits for ack by default
    """
    sendPacket = [0x55, opCode.value, opType.value, len(data)] + data
    if not self._connection:
      print('Warning: No robot connected')
      print('Tried to ' + opCode.name + ' ' + opType.name + ' with data: ' + str(data))
      print('Packet: ' + str(sendPacket))
      return
    #print('Packet: ' + str(sendPacket))
    self._connection.write(sendPacket)
    if waitForAck:
      self.waitForPacket(OPCODE.ACK, opType)

  def readPacket(self, blocking = False, timeout = None):
    """
    Reads a packet from the transport
      returns packet on success
      returns None if timeout or no packet available on non-blocking
    """
    if not self._connection:
      raise Exception('No robot connected - cannot read packet')
    if not blocking and self._connection.in_waiting <= 0:
      return None
    if timeout == None:
      timeout = self._BLOCKING_READ_TIMEOUT_SECS
    attempts = 0
    maxAttempts = self._READ_PACKET_TIMEOUT_SECS / timeout
    maxAttempts = math.floor(maxAttempts)
    header = None
    while attempts < maxAttempts:
      header = self._connection.read(4)
      attempts += 1
      if (len(header) == 4):
        if header[1] == OPCODE.ACK.value and header[2] == OPTYPE.NETWORK_KEEP_ALIVE.value:
          continue # network keepalive doesn't count as a useful packet
        break
    if (attempts == maxAttempts and len(header) == 0):
      return None # timeout
    if (len(header) != 4):
      raise Exception('Invalid header: ' + str(header))
    header = list(header)
    # Don't include start byte in packet
    header = header[1:]
    data = []
    dataLen = header[2]
    if (header[1] == OPTYPE.RPI_IMAGE.value):
      # One byte length doesn't work for big images so use as resolution flag
      if header[2] == IMRES.R640x480.value:
        dataLen = 640 * 480 * 3
      elif header[2] == IMRES.R1280x720.value:
        dataLen = 1280 * 720 * 3
      elif header[2] == IMRES.R1920x1088.value:
        dataLen = 1920 * 1088 * 3
      else:
        raise Exception('Invalid length for RPI_IMAGE packet: ' + str(header[2]))
    if dataLen > 0:
      data = self._connection.read(dataLen)
      data = list(data)
      if len(data) == 0:
        raise Exception('Timeout reading packet data')
    return header + data

  def waitForPacket(self, opCode, opType, timeout = None):
    """
    Blocks until a packet with a matching header is received
      throws an exception if an invalid or error packet is received
    """
    invalidReceiveOpCodes = [
      OPCODE.READ,
      OPCODE.WRITE,
      OPCODE.ERROR_INVALID_OP_CODE,
      OPCODE.ERROR_INVALID_PAYLOAD_SIZE,
      OPCODE.ERROR_INVALID_CHECKSUM,
      OPCODE.ERROR_NOT_IMPLEMENTED
    ]
    p = self.readPacket(True, timeout)
    if not p:
      raise Exception('Timeout waiting for packet')
    while OPCODE(p[0]) != opCode or OPTYPE(p[1]) != opType:
      if OPCODE(p[0]) in invalidReceiveOpCodes:
        raise Exception('Received invalid opcode: ' + OPCODE(p[0]).name)
      p = self.readPacket(True, timeout)
      print('Reading packet: ' + str(p))
      if not p:
        raise Exception('Timeout waiting for packet')

    return p

  def arm(self):
    """
    Puts the robot in UART control mode
    """
    self.writePacket(OPCODE.WRITE, OPTYPE.CONTROL_MODE, [1])

  def disarm(self):
    """
    Returns the robot to normal bluetooth operation
    """
    self.writePacket(OPCODE.WRITE, OPTYPE.CONTROL_MODE, [0])


def _sigint_handler(sig, frame):
  print('Received SIGINT... Stopping robot')
  rc = RoverController()
  rc.stopRobot()
  sys.exit(0)
