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
        
        plt.imsave('map_pic.png', np.flipud(self.grids), cmap=cm.gray)
        #for y in range(0, self.y_length, 1):
            
        #    for x in range(0, self.x_length, 1):
        #        print('|' + str(self.grids[self.y_length - 1 - y][x]), end = '')
            
        #        if x == (self.x_length - 1):
        #            print('|')
        
        img=mpimg.imread('map_pic.png')
        imgplot = plt.imshow(img)
        
        plt.show()
        plt.pause(0.001)
                    
    def mark_location(self, x, y, icon):
         if ((0 < x < self.x_length) & (0 < y < self.y_length)):
            self.grids[int(y)][int(x)] = int(icon)

    def mark_relative_location(self, currentX, currentY, distance, angle, icon):
        angle_rad = angle * m.pi/180    
        locationX = int(currentX + distance*m.cos(angle_rad))
        locationY = int(currentY + distance*m.sin(angle_rad))
        self.mark_location(locationX, locationY, icon)

    def mark_robot_pos(self, x_pos , y_pos, orientation):
        self.grids[self.grids == 1] = 0
        ori_rad = orientation * m.pi/180
        X = x_pos + 7*m.sqrt(2)*m.cos(ori_rad+m.pi*3/4)
        Y = y_pos + 7*m.sqrt(2)*m.sin(ori_rad+m.pi*3/4)

        for a in range(14):
            for b in range(14):
                self.mark_location(int(X+a*m.cos(ori_rad-m.pi/2)+b*m.cos(ori_rad)),int(Y+a*m.sin(ori_rad-m.pi/2)+b*m.sin(ori_rad)),1)
               
    def cone_error(self,x_pos,y_pos,orientation,angle):
        for a in range(40):
            for b in range (-angle,angle,1):
                self.mark_relative_location(x_pos,y_pos,a,orientation+b,0)
        