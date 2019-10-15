import math as m
import numpy
import enum
from micromelon import *

class Turn(enum.Enum):
    Left = 1
    Right = 2

class Position:
    def __init__(self, x, y, orientation, turn):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.turn = turn

    def update_position(distance, orientation):
        x = distance * m.cos(orientation);
        y = distance * m.sin(orientation);
        self.x += x
        self.y += y
        self.orientation = orientation

    def get_turn_priority():
        return self.turn.name;

def sweep():
    #Turn left 90 degrees. Then start the sweep.
    Motors.turnDegrees(-90, 15)
    measurements = [];
    for angle in range(0, 180, 10):
        measPair = (angle, angle);
        index = int(angle/10);
        measurements.append(measPair);
        print(measurements[index:index+1]);


#Choose the angle with the longest clearance.
#That means move towards the obstacle furthest away.
def get_direction(measurements):
    for x in range(0, measurements.length(), 1):
        
        
    
P = Position(0,0,0,Turn.Left)
sweep()
