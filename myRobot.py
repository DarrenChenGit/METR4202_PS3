import math as m
import numpy
import enum
import time 
from micromelon import *
from map import Map

class Turn(enum.Enum):
    Left = 1
    Right = 2

class myRobot:
    def __init__(self, x_size, y_size, orientation, turn):
        self.x = int(x_size/2)
        self.y = int(21)
        self.orientation = orientation
        self.turn = turn
        self.map = Map(x_size, y_size)
        self.map.mark_robot_pos(self.x,self.y,orientation)
        #self.map.mark_location(self.x,self.y,1)
        #self.map.grids[int(self.y)][int(self.x)] = 1
        self.dest = []

    def update_position(self, distance, orientation):
        angle_rad = orientation * m.pi/180
        x = distance * m.cos(angle_rad)
        y = distance * m.sin(angle_rad)
        self.x += x
        self.y += y
        self.orientation = orientation

    def get_turn_priority(self):
        return self.turn.name

    def sweep(self, increment = 5, coneAngle = 180):
        #Turn left 90 degrees. Then start the sweep.
        Motors.turnDegrees(-coneAngle/2,4)
        self.update_position(0,self.orientation-coneAngle)
        R.map.mark_robot_pos(self.x , self.y, self.orientation)
        time.sleep(3)
        measurements = []
        distance = 0
        for angle in range(0, coneAngle, increment):
            
            distance = Ultrasonic.read() +6
            measPair = (angle, distance)
            index = int(angle/increment)
            if (distance <= 40):
                self.map.mark_relative_location(self.x, self.y, distance, self.orientation, 2)
                measurements.append(measPair)
                #1.6 seems to be how much extra angle is needed to get correct angle
            Motors.turnDegrees(int(increment*1.6), 3)
            self.update_position(0,self.orientation-increment)
            R.map.mark_robot_pos(self.x , self.y, self.orientation)
            R.map.display_map()
            time.sleep(0.4)

        

    def move_forward(self, distance, angle):
        if angle == self.orientation:
            Motors.turnDegrees(angle - self.orientation, 5)
            time.sleep(3)
        Motors.moveDistance(distance)
        time.sleep(distance/15 + 1)
        self.map.mark_location(self.x, self.y, 0)
        self.map.mark_relative_location(self.x, self.y, distance, angle, 1)
        self.update_position(distance, angle)

    def set_dest(self, x, y):
        self.dest = [x, y]




#Choose the angle with the longest clearance.
#That means move towards the obstacle furthest away.

run = 1
R = myRobot(200, 200, m.pi/2, Turn.Left)
R.map.display_map()


if (run):
    
    rc = RoverController()
    rc.connectIP()
    R.sweep()
    #while 1:
    #    R.move_forward(2,m.pi/2)
    #    R.map.display_map()
    #    time.sleep(.5)
    
