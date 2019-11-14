import numpy
import cv2
import time
#import myRobot as mr
from pyzbar import pyzbar

from micromelon import *
# rc = RoverController()
# rc.connectIP()

# Global variable

# Select image resolution variable
#res = (1920,1080)
res = (1280,720)
#res = (640,480)


delta = 200
mid = int(res[0]/2)
R = (0, 0, 255)
G = (0, 255, 0) 
B = (255, 0, 0)

def getimage(res):
    if res[0] == 1920:
        image = Robot.getImageCapture(IMRES.R1920x1088)
    if res[0] == 1280:
        image = Robot.getImageCapture(IMRES.R1280x720)
    if res[0] == 640:    
        image = Robot.getImageCapture(IMRES.R640x480)
    image = image.astype(numpy.uint8)
    return image

def Canny(image):
    edges = cv2.Canny(image,25,125)
    return edges

def grayscale(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray

def binarize(image, ret):
    ret, binary = cv2.threshold(image,ret,255,cv2.THRESH_BINARY)
    return binary

def adbinarize(image):
    binary = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,115,1)
    return binary

def qrcodefunction(res):
    
    if res[0] == 1920:
        image = Robot.getImageCapture(IMRES.R1920x1088)
    if res[0] == 1280:
        image = Robot.getImageCapture(IMRES.R1280x720)
    if res[0] == 640:    
        image = Robot.getImageCapture(IMRES.R640x480)
    image = image.astype(numpy.uint8)
    
    # Scan normal image for qr code
    barcodes = pyzbar.decode(image)

    # Initialize variable
    x = 0
    y = 0
    w = 0
    h = 0

    # camera calibration
    knownwidth = 7.5 #cm
    knowndistance = 108 #cm
    knowpixelwidth = 128
    focal1080p = (knowpixelwidth*knowndistance)/knownwidth

    knowndistance640p = 30
    knownpixelheight640p = 167
    focal640p = (knownpixelheight640p*knowndistance640p)/knownwidth

    knowndis720p = 45
    knownpixh720p = 219
    focal720p = (knownpixh720p*knowndis720p)/knownwidth
    
    if res[0] == 1920:
        focal = focal1080p
    if res[0] == 1280:
        focal = focal720p
    if res[0] == 640:    
        focal = focal640p
        
    # binary image
    gray = grayscale(image)
    bi = adbinarize(gray)

    # Assign pixel location if found qr code
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect # x,y = starting point
        # w = pixel width of qr code
        # h = pixel height of qr code

    # Check if camera found qr code in normal image
    
    if x == 0: # no qr code found in normal image
        
        barcodes = pyzbar.decode(bi) # Try finding qr code in binary image
        
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            
        if x == 0: # no qr code found in binary image
            print('No qr code')

        else: # Found qr code in binary image
            #cv2.rectangle(image, (x, y), (x + w, y + h), G, 2)
            print('found qr code')
           
    else: # Found qr code in normal image
        #cv2.rectangle(image, (x, y), (x + w, y + h), G, 2)
        print('found qr code')
        
    
    dif = x + w/2 - mid
    
    # Check if the qr code is in middle area
    
    if x + (w/2) > mid + delta or x + (w/2) < mid - delta: 
        if x == 0: # No qr code found
            print('')
            print('')
        else: # qr code found outside the middle area
            #cv2.circle(image, (x + int(round(w/2)), y + int(round(h/2))), 5, R, -1)
            print('outside the area')
    else: # qr code found inside the middle area
        #cv2.circle(image, (x + int(round(w/2)), y + int(round(h/2))), 5, G, -1)
        print('Inside the area')

    distance = 0
    qrcentre = 0
    
    results = [distance,dif,qrcentre,mid-delta,mid+delta]
               
    if h == 0:
        print('')
        return results #[0,-res/2,0,mid-delta,mid+delta]
    else:
        qrcentre = x + w/2
        distance = (knownwidth*focal)/h
        results = [distance,dif,qrcentre,mid-delta,mid+delta]
        print('mid point of qr code =', x+(w/2))
        print('horizontal distance from center = ',dif)
        #print('x =', x)
        #print('y =', y)
        #print('w =', w)
        #print('h =', h)
        print('qr code distance =', distance)
        #print('')
        #print('')
        #print('')
        return results
        
    #Darren: Adding some code so function returns some value.
    #if x == 0: #No QR
     #   return None

    #else:
     #   return distance
     
def increase_Bri(res):
    results = qrcodefunction(res)
    if results[0] != 0:
        return results
    else:
        if res[0] == 1920:
            image = Robot.getImageCapture(IMRES.R1920x1088)
        if res[0] == 1280:
            image = Robot.getImageCapture(IMRES.R1280x720)
        if res[0] == 640:    
            image = Robot.getImageCapture(IMRES.R640x480)
        image = image.astype(numpy.uint8)

        alpha = 1.2 # Simple contrast control
        beta = 3    # Simple brightness control
        image = cv2.convertScaleAbs(image,alpha,beta) # Increasse brightness and contrast of normal image (BNI)

        # Scan BNI for qr code
        barcodes = pyzbar.decode(image)

        # Initialize variable
        x = 0
        y = 0
        w = 0
        h = 0

        # camera calibration
        knownwidth = 7.5 #cm
        knowndistance = 108 #cm
        knowpixelwidth = 128
        focal1080p = (knowpixelwidth*knowndistance)/knownwidth

        knowndistance640p = 30
        knownpixelheight640p = 167
        focal640p = (knownpixelheight640p*knowndistance640p)/knownwidth

        knowndis720p = 45
        knownpixh720p = 219
        focal720p = (knownpixh720p*knowndis720p)/knownwidth
    
        if res[0] == 1920:
            focal = focal1080p
        if res[0] == 1280:
            focal = focal720p
        if res[0] == 640:    
            focal = focal640p
        
        # brighten binary image (BBI)
        gray = grayscale(image)
        bi = adbinarize(gray)

        # Assign pixel location if found qr code
        for barcode in barcodes:
             (x, y, w, h) = barcode.rect # x,y = starting point
            # w = pixel width of qr code
            # h = pixel height of qr code

        # Check if camera found qr code in BNI 
    
        if x == 0: # no qr code found in BNI
        
            barcodes = pyzbar.decode(bi) # Try finding qr code in BBI
        
            for barcode in barcodes:
                (x, y, w, h) = barcode.rect
            
            if x == 0: # no qr code found in BBI
                print('No qr code (Bright)')

            else: # Found qr code in BBI
                #cv2.rectangle(image, (x, y), (x + w, y + h), G, 2)
                print('found qr code (Bright)')
           
        else: # Found qr code in BNI
            #cv2.rectangle(image, (x, y), (x + w, y + h), G, 2)
            print('found qr code (Bright)')
        
    
        dif = x + w/2 - int(res[0]/2)
    
        # Check if the qr code is in middle area
    
        if x + (w/2) > mid + delta or x + (w/2) < mid - delta: 
            if x == 0: # No qr code found
                print('')
                #print('')
            else: # qr code found outside the middle area
                #cv2.circle(image, (x + int(round(w/2)), y + int(round(h/2))), 5, R, -1)
                print('outside the area')
        else: # qr code found inside the middle area
            #cv2.circle(image, (x + int(round(w/2)), y + int(round(h/2))), 5, G, -1)
            print('Inside the area')

        distance = 0
        qrcentre = 0
    
        results = [distance,dif,qrcentre,mid-delta,mid+delta]
               
        if h == 0:
            print('')
            #cv2.imshow('assd',image)
            #cv2.waitKey(0)
            return results #[0,-res/2,0,mid-delta,mid+delta]
        else:
            qrcentre = x + w/2
            distance = (knownwidth*focal)/h
            results = [distance,dif,qrcentre,mid-delta,mid+delta]
            print('mid point of qr code =', x+(w/2))
            print('horizontal distance from center = ',dif)
            #print('x =', x)
            #print('y =', y)
            #print('w =', w)
            #print('h =', h)
            print('qr code distance =', distance)
            #print('')
            #print('')
            print('')
            #cv2.imshow('assd',image)
            #cv2.waitKey(0)
            return results

        
def middlearea(image, res, delta):
    
    midd = int(res[0]/2)
    
    maxrange = midd + delta
    minrange = midd - delta

    cv2.line(image, (midd,0), (midd,res[1]), G, 1)
    cv2.line(image, (minrange,0), (minrange,int(res[1])), B, 1)
    cv2.line(image, (maxrange,0), (maxrange,int(res[1])), B, 1)

# def main():
#     i = 0
#     image = 0
#     image = getimage(res)
#     qrcodefunction(image)
#     middlearea(image, res, delta)
#     cv2.imshow('image', image)
#     cv2.waitKey(1000)
#def main():
 #   image = getimage(res)
  #  qrcodefunction(image)

#while True:
 #   main()
  #  time.sleep(2)
