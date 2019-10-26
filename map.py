import math as m
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.image as mpimg   
import numpy as np

class Map:
    def __init__(self, x_length, y_length):
        self.x_length = x_length
        self.y_length = y_length
        self.grids = np.array([[ 0 for x in range(0, x_length, 1)] for y in range(0, y_length, 1)])
        plt.ion()

    #becareful of indexing, it is y, x.
    def display_map(self):
        
        plt.imsave('map.png', np.flipud(self.grids), cmap=cm.gray)
        #for y in range(0, self.y_length, 1):
            
        #    for x in range(0, self.x_length, 1):
        #        print('|' + str(self.grids[self.y_length - 1 - y][x]), end = '')
            
        #        if x == (self.x_length - 1):
        #            print('|')
        
        img=mpimg.imread('map.png')
        imgplot = plt.imshow(img)
        
        plt.show()
        plt.pause(0.001)
                    
    def mark_location(self, x, y, icon):
        self.grids[y][x] = icon

    def mark_relative_location(self, currentX, currentY, distance, angle, icon):
        angle_rad = angle * m.pi/180
        locationX = int(currentX + distance*m.cos(angle_rad))
        locationY = int(currentY + distance*m.sin(angle_rad))
        self.mark_location(locationX, locationY, icon)
