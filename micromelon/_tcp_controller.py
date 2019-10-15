import socket
import select
import errno


class SerialTCPConnection:
  """
  A byte level api for RoverController to use to control networked Micromelon backpacks
  The Backpack must be running a server script
  """
  def __init__(self, address = '192.168.4.1', port = 4202, timeout = 3.0):
    self._recvBuffer = []
    self._port = port
    self._recvTimeout = timeout
    self._in_waiting = 0
    self._isOpen = False
    self._address = address
    self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # self._sock.setblocking(0)

  @property
  def port(self):
    return self._port

  @port.setter
  def port(self, p):
    if p < 1024 or p > 65535:
      raise ValueError('TCP port must be between 1024 and 65535')
    if self._isOpen:
      raise Exception('Connection already open to {}:{}'.format(self._address, self._port))
    self._port = p

  @property
  def address(self):
    return self._address

  @address.setter
  def address(self, add):
    if self._isOpen:
      raise Exception('Connection already open to {}:{}'.format(self._address, self._port))
    self._address = add

  @property
  def in_waiting(self):
    if not self._isOpen:
      return 0
    self._sock.settimeout(0)
    try:
      data = self._sock.recv(4096)
      self._recvBuffer.extend(list(data))
    except socket.error as e:
      if e.errno != errno.EAGAIN and e.errno != errno.EWOULDBLOCK:
        raise e
    self._sock.settimeout(self._recvTimeout)
    return len(self._recvBuffer)

  def open(self):
    self._sock.connect((self._address, self._port))
    self._sock.settimeout(self._recvTimeout)
    self._isOpen = True

  def close(self):
    if self._isOpen:
      self._sock.shutdown(socket.SHUT_RDWR)
      self._sock.close()
      self._isOpen = False

  def flushInput(self):
    self._recvBuffer = []
    try:
      data = self._sock.recv(1024)
    except socket.timeout as e:
      return # don't care about timeout when flushing
    except socket.error as e:
      raise e

  def flushOutput(self):
    return # Only here to mimic serial API

  def read(self, dataLen):
    result = self._getFromRecvBuffer(dataLen)
    while len(result) == 0:
      try:
        data = self._sock.recv(dataLen)
        self._recvBuffer.extend(list(data))
      except socket.timeout as e:
        return []
      result = self._getFromRecvBuffer(dataLen)
    return result

  def _getFromRecvBuffer(self, dataLen):
    if len(self._recvBuffer) >= dataLen:
      retVal = self._recvBuffer[0 : dataLen]
      self._recvBuffer = self._recvBuffer[dataLen:]
      return retVal
    else:
      return []

  def write(self, data):
    while len(data):
      try:
        sent = self._sock.send(bytes(data))
        data = data[sent:]
      except socket.error as e:
        if e.errno != errno.EAGAIN:
          raise e
        select.select([], [self._sock], [])  # This blocks until we can write more
