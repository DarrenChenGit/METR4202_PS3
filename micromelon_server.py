import socket
import select
import errno
import sys
import time

import numpy
from picamera.array import PiRGBArray
from picamera import PiCamera

from micromelon import *
from micromelon.comms_constants import MicromelonOpCode as OPCODE, MicromelonType as OPTYPE, MicromelonImageResolution as IMRES, tupleForResolution

KEEP_ALIVE_INTERVAL_SECS = 5.0

port = 4202
if len(sys.argv) == 2:
  port = int(sys.argv[1])

camera = PiCamera()
res = tupleForResolution(IMRES.R640x480)
camera.resolution = res
rawCapture = PiRGBArray(camera)
print('Starting')
# Allow camera warmup
time.sleep(1)
print('Camera initialised')

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Don't want to hang onto port after closing
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
print('Listening on port: ' + str(port))
sock.bind(('', port))
sock.listen(1)

rc = RoverController()
try:
  rc.connectSerial()
except Exception as e:
  print('Could not connect to robot serial, only backpack functions available')
  print('(Camera only mode)')

def captureImage(res):
  global rawCapture
  global camera
  rawCapture.truncate(0)
  if not isinstance(res, IMRES):
    res = IMRES(res)
  
  camera.resolution = tupleForResolution(res)

  camera.capture(rawCapture, format="bgr", use_video_port=True)
  image = rawCapture.array
  return image.astype(numpy.uint8)


def sendToNetwork(conn, data):
  totalSent = 0
  while len(data):
    try:
      sent = conn.send(bytes(data))
      totalSent += sent
      data = data[sent:]
    except socket.error as e:
      if e.errno != errno.EAGAIN:
        print('Error: ' + str(e))
        return -1
      select.select([], [conn], [])  # This blocks until we can write more
  return totalSent

while True:
  print('Waiting for connection')
  connection, clientAddress = sock.accept()
  print('Got connection from {}'.format(clientAddress))
  connection.setblocking(0)
  networkRecvBuffer = []
  lastKeepAlive = time.time()
  while True:
    t = time.time()
    if t - lastKeepAlive > KEEP_ALIVE_INTERVAL_SECS:
      if sendToNetwork(connection, [0x55, OPCODE.ACK.value, OPTYPE.NETWORK_KEEP_ALIVE.value, 0]) == -1:
        break
      lastKeepAlive = time.time()

    # Check for packets on serial and send to network
    botPacket = rc.readPacket(blocking=False)
    if botPacket != None:
      lastKeepAlive = time.time()
      if sendToNetwork(connection, [0x55] + botPacket) == -1:
        break

    # Non-blocking check for packets on network to send to serial
    data = None
    try:
      data = connection.recv(4096)
    except socket.error as e:
      if e.errno != errno.EAGAIN and e.errno != errno.EWOULDBLOCK:
        # Error other than no data immediately available
        break

    if (data != None and len(data) > 0):
      networkRecvBuffer.extend(list(data))
    # At least a packet header available
    if len(networkRecvBuffer) >= 4:
      header = networkRecvBuffer[1:4]
      if len(networkRecvBuffer) < header[2] + 4:
        # Received header but not data yet
        continue
      # set data based on length field of header
      data = networkRecvBuffer[4:4 + header[2]]
      networkRecvBuffer = networkRecvBuffer[4 + header[2]:]
      if header[0] == OPCODE.READ.value and header[1] == OPTYPE.RPI_IMAGE.value:
        image = captureImage(data[0])
        image = numpy.reshape(image, numpy.prod(image.shape))
        lastKeepAlive = time.time()
        if sendToNetwork(connection,
            bytes([0x55, OPCODE.ACK.value, OPTYPE.RPI_IMAGE.value, data[0]]) + bytes(image)) == -1:
          break

      else:
        rc.writePacket(OPCODE(header[0]), OPTYPE(header[1]), data, waitForAck=False)      
  
  print('Lost connection to {}'.format(clientAddress))
  connection.close()
