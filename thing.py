import numpy
import cv2
import time
from pyzbar import pyzbar

from micromelon import *
rc = RoverController()
rc.connectIP()

def getimage():
    #image = Robot.getImageCapture(IMRES.R640x480)
    image = Robot.getImageCapture(IMRES.R1920x1088)
    #image = Robot.getImageCapture(IMRES.R1280x720)
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

def findbarcode(image):
    # Find the barcodes in the image
    barcodes = pyzbar.decode(image)
    
    for barcode in barcodes:
        (x, y, w, h)= barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h)
            , (0, 255, 0), 2)
        
        dif = x + w/2 - 960
        print('horizontal distance from center = ',dif)
        print('x =', x)
        print('y =', y)
        print('w =', w)
        print('h =', h)
        print('mid point of qr code =', x+(w/2))
        
def findbarcenter(image):
    barcodes = pyzbar.decode(image)
    
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        if x + (w/2) >1160 or x + (w/2) < 760:
            cv2.circle(image, (x + int(round(w/2)), y + int(round(h/2))), 5, (0, 0,255), -1)
            print('outside the area')
        else:
            cv2.circle(image, (x + int(round(w/2)), y + int(round(h/2))), 5, (0, 255,0), -1)
            print('Inside the area')

def finddistance(image):
    
    knownwidth = 7 #cm
    knowndistance = 41 #cm
    knowpixelwidth = 320
    focal = (knowpixelwidth*knowndistance)/knownwidth
    
    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        distance = (knownwidth*focal)/w
        print('distance =', distance)
        
def main():
    
    image = getimage()
##    gray = grayscale(image)
##    gray = cv2.GaussianBlur(gray, (5, 5), 0)
##    
##    kernel = numpy.ones((2,2),numpy.uint8)
##    kernel2 = numpy.ones((5,5),numpy.uint8)
##    
##    thg = adbinarize(gray)
##    
##    closing = cv2.morphologyEx(thg, cv2.MORPH_CLOSE, kernel)
##    dilate = cv2.dilate(closing, kernel,iterations = 3)
##    
##    
##    contours, hierarchy = cv2.findContours(dilate,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    findbarcode(image)
##    for contour in contours:
##        if cv2.contourArea(contour)>1000:
##            cv2.drawContours(image,contour,-1, (0,0,255),2)
    
    findbarcenter(image)
    finddistance(image)
    
    delta = 200
    maxrange = 960 + delta
    minrange = 960 - delta
    
    cv2.line(image, (960,0), (960,1080), (0,0, 255), 1)
    cv2.line(image, (minrange,0), (minrange,1080), (255,0, 0), 1)
    cv2.line(image, (maxrange,0), (maxrange,1080), (255,0, 0), 1)
    
    cv2.imshow('image', image)
    cv2.waitKey(0)

main()    
    


    
