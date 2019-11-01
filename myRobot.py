import math as m
import numpy
import enum
import time 
from map import Map
from micromelon import *

run = 0

class Turn(enum.Enum):
    Left = 1
    Right = 2

class State(enum.Enum):
    Explore = 0
    QRFound = 1
    Returning = 2

class myRobot:
    def __init__(self, turn, x_size = 200, y_size = 200, speed = 6, turnSpeed = 4, orientation = 90):
        self.x = int(x_size/2)
        self.y = int(21)
        self.orientation = orientation #Orientation w.r.t to world coordinates.
        self.speed = speed #How fast robot move forward/backward
        self.turnSpeed = turnSpeed #How fast robot should turn.
        self.turn = turn
        self.map = Map(x_size, y_size)
        self.map.mark_robot_pos(self.x,self.y,orientation)
        self.state = State.Explore
        #self.map.mark_location(self.x,self.y,1)
        #self.map.grids[int(self.y)][int(self.x)] = 1
        self.dest = []

    #Integrated function that uses Motors and Map to update location.
    #Convention:
    #+Angle - Right turn.
    #-Angle - Left turn.
    def turn_robot(self, degrees):
        self.orientation -= degrees
        if run:
            Motors.turnDegrees(degrees, self.turnSpeed)
            time.sleep(0.5)
        #Mark the robot position.
        self.map.mark_robot_pos(self.x , self.y, self.orientation)

    def move_forward(self, distance):
        #Calculate where the robot should be if it moved.
        destX = self.x + distance * m.cos(m.radians(self.orientation))
        destY = self.y + distance * m.sin(m.radians(self.orientation))

        #Only move if it is within bounds.
        if ((0 < destX < self.map.x_length) and (0 < destY < self.map.y_length)):
            if (run):
                Motors.moveDistance(distance)
                time.sleep(distance/self.speed + 1)
            self.update_position(distance, self.orientation)
            self.map.mark_robot_pos(self.x , self.y, self.orientation)
            
        #Out of bounds movement.
        else:
            print("Robot will move out of bounds if moved!")


    def update_position(self, distance, orientation):
        angle_rad = m.radians(orientation)
        x = distance * m.cos(angle_rad)
        y = distance * m.sin(angle_rad)
        self.x += x
        self.y += y
        self.orientation = orientation
        self.map.mark_location(self.x, self.y, 1)

    def get_turn_priority(self):
        return self.turn.name

    def sweep(self, increment = 5, coneAngle = 180):
        #Turn left 90 degrees. Then start the sweep.
        Motors.turnDegrees(-coneAngle/2,4)
        time.sleep(3)
        self.update_position(0, self.orientation+coneAngle/2)
        R.map.mark_robot_pos(self.x , self.y, self.orientation)
        
        measurements = []
        distance = 0
        angle = 0
        skip_angle = 20
        skip_set = 0
        while 1:
            if angle >= coneAngle:
                break
            
            distance = Ultrasonic.read() + 6
            measPair = (angle, distance)
            index = int(angle/increment)
            if (distance <= 40):
                if (skip_set == 0):
                    self.map.mark_relative_location(self.x, self.y, distance, self.orientation, 2)
                    measurements.append(measPair)
                    Motors.turnDegrees(7.5, 3)
                    self.update_position(0, self.orientation-increment)
                    R.map.mark_robot_pos(self.x , self.y, self.orientation)
                    angle = angle +5
                else:
                    skip_set = 0
                    self.map.cone_error(self.x, self.y,self.orientation,skip_angle-increment)
                    Motors.turnDegrees(skip_angle-increment, 3)
                    self.update_position(0,self.orientation-skip_angle+increment)
                    R.map.mark_robot_pos(self.x , self.y, self.orientation)
                    angle = angle + skip_angle-increment


                #1.6 seems to be how much extra angle is needed to get correct angle
            else :
                self.map.cone_error(self.x, self.y,self.orientation,skip_angle)
                Motors.turnDegrees(7.5, 3)
                self.update_position(0,self.orientation-increment)
                R.map.mark_robot_pos(self.x , self.y, self.orientation)
                angle = angle + increment
                skip_set = 1
            
            R.map.display_map()
            time.sleep(.8)

    #Sweep using the left IR sensor.
    def ir_sweep(self, coneAngle, increment):
        offsetAngle = 90 - coneAngle/2
        Motors.turnDegrees(offsetAngle, 4)
        time.sleep(5)

        #Update position and mark robot position on map.
        self.update_position(0, self.orientation + offsetAngle)
        R.map.mark_robot_pos(self.x , self.y, self.orientation)

        measurements = []
        for angle in range(0, coneAngle + increment, increment):
            IR_reading = IR.readLeft() + 2
            if (IR_reading < 100):
                print(IR_reading)
                measurements.append(IR_reading)
                self.map.mark_relative_location(self.x, self.y, IR_reading, self.orientation, 2)
                Motors.turnDegrees(increment, 4)
                time.sleep(0.4)

    def set_dest(self, x, y):
        self.dest = [x, y]

    #Avoid an obstacle by turning left/right then moving away.
    def avoid_obstacle(self):
        leftReading = IR.readLeft()
        rightReading = IR.readRight()
        if (leftReading > 10):
            self.turn = Turn.Left
            self.turn_robot(-90)
            self.move_forward(leftReading/2)
        
        elif (rightReading > 10):
            self.turn = Turn.Right
            self.turn_robot(90)
            self.move_forward(rightReading/2)
        
        else:
            #We might wanna reverse.
        
        
        
        
        
        
        

    def face_objective(self):
        if (not self.dest):
            print("No destination has been set!")
            return
        meas = self.map.get_distance_and_angle(self.x, self.y, self.dest[0], self.dest[1])
        Motors.turnDegrees(meas[1])

    def move_to_objective(self):
        return 0

#Choose the angle with the longest clearance.
#That means move towards the obstacle furthest away.


R = myRobot(Turn.Left)
#R.map.display_map()


if (run):
    
    rc = RoverController()
    rc.connectIP()
    #R.ir_sweep(120, 10)
    #while 1:
    #    R.move_forward(2,m.pi/2)
    #    R.map.display_map()
    #    time.sleep(.5)
    
