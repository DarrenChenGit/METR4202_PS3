import math as m
import numpy
import enum
import time 
__import__('map')
from micromelon import *
from map import Map

class Turn(enum.Enum):
    Left = 1
    Right = 2

class myRobot:
    def __init__(self, x, y, orientation, turn):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.turn = turn
        self.map = Map(20,20)

    def update_position(distance, orientation):
        x = distance * m.cos(orientation);
        y = distance * m.sin(orientation);
        self.x += x
        self.y += y
        self.orientation = orientation

    def get_turn_priority():
        return self.turn.name;

    def sweep(self):
        #Turn left 90 degrees. Then start the sweep.
        Motors.turnDegrees(-90, 2)
        time.sleep(5)
        measurements = [];
        distance = 0;
        for angle in range(0, 180, 10):
            distance = Ultrasonic.read();
            time.sleep(0.5)
            measPair = (angle, distance);
            index = int(angle/10);
            if (distance <= 40):
                self.map.mark_relative_location(self.x, self.y, distance/2, angle);
                measurements.append(measPair);
            Motors.turnDegrees(10, 10);
            print(distance);
            time.sleep(1)


#Choose the angle with the longest clearance.
#That means move towards the obstacle furthest away.
R = myRobot(0, 0, 0, Turn.Left);
rc = RoverController()
rc.connectIP()
run = 0;

if (run):
    R.sweep()
    R.map.display_map()
    
while (1):
    d = Ultrasonic.read()
    print(d);
    time.sleep(1)
