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
        if 1:
        
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
        if ((0 < x < self.x_length) and (0 < y < self.y_length)):
            self.grids[int(y)][int(x)] = int(icon)
            

    def mark_relative_location(self, currentX, currentY, distance, angle, icon):
        angle_rad = m.radians(angle)  
        locationX = int(currentX + distance*m.cos(angle_rad))
        locationY = int(currentY + distance*m.sin(angle_rad))
        self.mark_location(locationX, locationY, icon)
        return (locationX, locationY)

    def mark_robot_pos(self, x_pos , y_pos, orientation):
        self.grids[self.grids == 1] = 0
        ori_rad = orientation * m.pi/180
        X = x_pos + 7*m.sqrt(2)*m.cos(ori_rad+m.pi*3/4)
        Y = y_pos + 7*m.sqrt(2)*m.sin(ori_rad+m.pi*3/4)

        for a in range(14):
            for b in range(14):
                self.mark_location(int(X+a*m.cos(ori_rad-m.pi/2)
                +b*m.cos(ori_rad)),int(Y+a*m.sin(ori_rad-m.pi/2)+b*m.sin(ori_rad)),1)
               
    def cone_error(self,x_pos,y_pos,orientation,angle):
        array = []
        for a in range(40):
            for b in range (-angle,angle,1):
                array.append(self.mark_relative_location(x_pos,y_pos,a,orientation+b,0))
        return array

    #Function to calculate distance between 2 points and get the relative angle.
    #Between two points with respect to 90 degrees of world coordinate.
    def get_dist_angle(self, self_x, self_y, dest_x, dest_y):
        returnVal = [] #Empty list to be filled.
        x = dest_x - self_x
        y = dest_y - self_y
        euclidDist = m.sqrt(x*x + y*y) #Calc distance
        returnVal.append(euclidDist)
        if (x):
            angle = m.degrees(m.atan(x/y)) #Calc angle.
        if x == 0:
            if y >= 0:
                returnVal.append(90)

            else:
                returnVal.append(-90)

            return returnVal

        if (y < 0 and x > 0):
            returnVal.append(angle)

        elif (x < 0 and y >= 0):
            returnVal.append(90- angle)

        elif (x >= 0 and y >= 0):
            returnVal.append(90-angle)

        else:
            returnVal.append(-90-angle)
         
        return returnVal

    #Checks to see if a point is within a square radius.
    #Param: center_x, center_y is the center of the square
    #x, y is the point to check if within square
    #radius is the length of 1/2 side of the square.
    def is_point_in_sqr_radius(self, center_x, center_y, x, y, radius = 30):
        if ((center_x - radius) <= x <= (center_x + radius) and
        (center_y - radius) <= y <= (center_y + radius)):
            return True

        else: 
            return False

        
       
