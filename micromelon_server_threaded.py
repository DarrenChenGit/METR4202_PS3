import socket
import select
import errno
import sys
import time
import threading

import numpy
from picamera.array import PiRGBArray
from picamera import PiCamera

from micromelon import *
from micromelon.comms_constants import MicromelonOpCode as OPCODE, MicromelonType as OPTYPE, MicromelonImageResolution as IMRES, tupleForResolution

KEEP_ALIVE_INTERVAL_SECS = 5.0

port = 4202
if len(sys.argv) == 2:
  port = int(sys.argv[1])

class PiVideoStream:
  def __init__(self, resolution=IMRES.R320x240):
    # initialize the camera and stream
    self.camera = PiCamera()
    self.camera.framerate = 12
    self.camera.resolution = tupleForResolution(resolution)

    self.rawCapture = PiRGBArray(self.camera)

    self.res = resolution
    self.frameAvailable = threading.Event()
    self.captureThread = None
    self._frame = None
    self.stopped = True

  def start(self, resolution):
    if not isinstance(resolution, IMRES):
      res = IMRES(resolution)
    if self.captureThread != None:
      if self.captureThread.is_alive():
        if self.res == resolution and not self.stopped:
          return
        self.stop()
    self.res = resolution
    self.camera.resolution = tupleForResolution(resolution)
    # start the thread to read frames from the video stream
    self.stopped = False
    self.frameAvailable.clear()
    self.captureThread = threading.Thread(target=self._capture, args=())
    self.captureThread.start()

  def _capture(self):
    stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
    lastT = 0.0
    for f in stream:
      # grab the frame from the stream and clear the stream in
      # preparation for the next frame
      self._frame = f.array.astype(numpy.uint8)
      self.rawCapture.truncate(0)
      self.frameAvailable.set()
 
      #t = time.time()
      #print('Seconds per frame: ' + str(t - lastT))
      #lastT = t

      # if the thread indicator variable is set, stop the thread
      # and resource camera resources
      if self.stopped:
        return

  def readFrame(self):
    # return the frame most recently read
    self.frameAvailable.wait()
    self.frameAvailable.clear()
    return self._frame
 
  def stop(self):
    # indicate that the thread should be stopped
    self.stopped = True
    if self.captureThread != None:
      self.captureThread.join()

  def captureImage(self, res):
    self.start(res)
    return self.readFrame()

cameraWrapper = PiVideoStream()

print('Starting')

rc = RoverController()
rc.setReadTimeout(1.0)
try:
  rc.connectSerial()
except Exception as e:
  print('Could not connect to robot serial, only backpack functions available')
  print('(Camera only mode)')

rc.setReadTimeout(120.0)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Don't want to hang onto port after closing
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
print('Listening on port: ' + str(port))
sock.bind(('', port))
sock.listen(1)



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

# Network socket
connection = None

def monitorNetwork():
  global rc
  global connection
  networkRecvBuffer = []
  try:
    while True:
      data = connection.recv(4096)
      if len(data) == 0:
        raise Exception('Connection died')
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
        image = cameraWrapper.captureImage(data[0])
        image = numpy.reshape(image, numpy.prod(image.shape))
        if sendToNetwork(connection,
            bytes([0x55, OPCODE.ACK.value, OPTYPE.RPI_IMAGE.value, data[0]]) + bytes(image)) == -1:
          raise Exception('Failed to send to socket')
      else:
        rc.writePacket(OPCODE(header[0]), OPTYPE(header[1]), data, waitForAck=False)
  finally:
    return


def monitorSerial():
  global rc
  global connection
  # Check for packets on serial and send to network
  while True:
    botPacket = rc.readPacket(blocking=True)
    if botPacket != None:
      lastKeepAlive = time.time()
      if connection and sendToNetwork(connection, [0x55] + botPacket) == -1:
        break
  return


cameraWrapper.start(IMRES.R320x240)

serialThread = threading.Thread(target=monitorSerial, args=())
serialThread.start()

while True:
  print('Waiting for connection')
  connection, clientAddress = sock.accept()
  print('Got connection from {}'.format(clientAddress))
  connection.setblocking(1) # 1 for blocking, 0 for non-blocking

  networkThread = threading.Thread(target=monitorNetwork, args=())
  networkThread.start()

  networkThread.join()

  connection.close()
  connection = None
  print('Lost connection to {}'.format(clientAddress))
