import math as m
import numpy
import enum
import time
import thing
from map import Map
from micromelon import *

run = 1
res = (1920,1080)
#res = (1280,720)
#res = (640,480)

delta = 50
mid = int(res[0]/2)


class Turn(enum.Enum):
    Left = 1
    Right = 2
    Back = 3
    BackLeft = 4

class State(enum.Enum):
    Explore = 0
    Returning = 2

class myRobot:
    def __init__(self, turn, x_size = 200, y_size = 200, speed = 2, turnSpeed = 4, orientation = 90):
        self.x = int(x_size/2)
        self.y = int(21)
        self.orientation = orientation #Orientation w.r.t to world coordinates.
        self.speed = speed #How fast robot move forward/backward
        self.turnSpeed = turnSpeed #How fast robot should turn.
        self.turn = turn
        self.map = Map(x_size, y_size)
        self.map.mark_robot_pos(self.x,self.y,orientation)
        self.state = State.Explore
        self.dest = [90,195]
        self.obstacles = []
        self.QRFound = False
        self.turn_state = 0

#ROBOT SETTINGS
    def set_dest(self, x, y):
        self.dest = [x, y]

    def set_turn_speed(self, speed):
        self.turnSpeed = speed

    def set_speed(self, speed):
        self.speed = speed

#GET FUNCTIONS
    def get_pos(self):
        print("Robot at ", self.x, " ", self.y, " ", "orientation: ", self.orientation)
        return (self.x, self.y, self.orientation)
        
#MOVEMENT FUNCTIONS

    #Integrated function that uses Motors and Map to update location.
    #Convention:
    #+Angle - Right turn.
    #-Angle - Left turn.
    def turn_robot(self, degrees):
        delay = ((2 * m.pi * 8) * (abs(degrees)/360)) / self.turnSpeed 
        self.orientation -= degrees
        if run:
            if abs(degrees) == 5:
                degrees = degrees*1.2
            Motors.turnDegrees(int(degrees), self.turnSpeed)
            time.sleep(delay + 0.4)

        #We can calibrate the stuff.
        #Mark the robot position.
        self.map.mark_robot_pos(self.x , self.y, self.orientation)

    def move_forward(self, distance):
        ##Calculate where the robot should be if it moved.
        # destX = self.x + distance * m.cos(m.radians(self.orientation))
        # destY = self.y + distance * m.sin(m.radians(self.orientation))
        Motors.moveDistance(distance)
        time.sleep(abs(distance)/self.speed)
        self.update_position(distance, self.orientation)
        self.map.mark_robot_pos(self.x , self.y, self.orientation)

    def ir_pass(self, turn):
        if turn == 1: #left
            while IR.readLeft() < 15:
                #if Ultrasonic.read() > 7:
                #    self.move_forward(2)
                self.move_forward(3)
            self.move_forward(10)
            #x = 0
            #while x < 10:
            #    if Ultrasonic.read() < 7:
            #        self.move_forward(2)
            #        x = x+2
        elif turn == 2:
            while IR.readRight() < 15:
            #    if Ultrasonic.read() > 7:
                self.move_forward(3)
            self.move_forward(10)
            #x = 0
            #while x < 10:
            #    if Ultrasonic.read() < 7:
            #        self.move_forward(2)
            #        x = x+2


    #Avoid an obstacle by turning left/right then moving away.
    def avoid_obstacle(self):
       while(1):
        direction = self.get_turn_priority()
        if direction == Turn.Left:
            self.turn_robot(-90)
            sensor = Turn.Right
        if direction == Turn.Right:
            self.turn_robot(90)
            sensor = Turn.Left
        if direction == Turn.Back:
            self.move_forward(-15)
            self.get_turn_priority()
        
        while Ultrasonic.read() > 5:
            if self.IR_read(sensor) > 10:
                a = 0
                while Ultrasonic.read() > 5:
                    self.move_forward(2)
                    a = a +2
                    if a >= 10:
                        return
                return
            self.move_forward(2)
        return
                    
    #Determine which IR sensor to read based on which direction we turn.
    def IR_read(self,sensor):
        if sensor == Turn.Right:
            return IR.readRight()
        if sensor == Turn.Left:
            return IR.readLeft()

    #Decide what is the best direction to turn at the current state.
    def get_turn_priority(self):
        
        leftReading = IR.readLeft()
        rightReading = IR.readRight()

        if leftReading < 20:
            if rightReading  < 20:
                #Not enough clearance for both.
                self.turn_state == Turn.Back
                return Turn.Back
            else:
                if self.turn_state == Turn.Left:
                    self.turn_state = Turn.Back
                    return Turn.Back
                else:
                    self.turn_state = Turn.Right
                    return Turn.Right
        else: 
            if rightReading > 20:
                if self.x > self.dest[0]:
                    self.turn_state = Turn.Left
                    return Turn.Left
                else:
                    self.turn_state = Turn.Right
                    return Turn.Right
            else: 
                if self.turn_state == Turn.Right:
                    self.turn_state = Turn.Back
                    return Turn.Back
                else: 
                    self.turn_state = Turn.Left
                    return Turn.Left

    def move_cam_to_mid(self, results):
    # move camera to position rover straight at qrcode
    # results = [dis,dif,qrcen,min,max]
        dif = results[1]
        qrcentre = results[2]
        #minrange = results[3]
        #maxrange = results[4]
        while 1:
            results = thing.qrcodefunction(res)
            qrcentre = results[2]
            if qrcentre > mid+delta:
                self.turn_robot(7)
                

            elif qrcentre < mid-delta:
                self.turn_robot(-7)

            else:
                x = self.x + results[0] * m.cos(self.orientation)
                y = self.y + results[0] * m.sin(self.orientation)
                self.dest = [x, y]
                break
##        if results[1] > 0: # qrcentre is left side of camera
##            while qrcentre > mid+delta: # qrcentre outside mid area
##                print('Want to turn left')
##                self.turn_robot(10) # Left turn
##                results = thing.qrcodefunction(res)
##                qrcentre = results[2]
##                x = self.x + results[0] * m.cos(self.orientation)
##                y = self.y + results[0] * m.sin(self.orientation)
##                self.dest = [x, y]
##                
##        elif results[1] < 0 : # qrcentre is right side of camera
##            while qrcentre < mid-delta: # qrcentre outside mid area
##                self.turn_robot(-10) # Right turn
##                results = thing.qrcodefunction(res)
##                qrcentre = results[2]
##                x = self.x + results[0]*m.cos(self.orientation)
##                y = self.y + results[0]*m.sin(self.orientation)
##                self.dest = [x, y]
##                
##        else: # No qr code
##            return None
##        
##        if mid - delta < qrcentre < mid + delta:
##            dis = results[0]
##            R.move_forward(dis/2)
            
#MAPPING FUNCTIONS
    def update_position(self, distance, orientation):
        angle_rad = m.radians(orientation)
        x = distance * m.cos(angle_rad)
        y = distance * m.sin(angle_rad)
        self.x += x
        self.y += y
        self.orientation = orientation
        self.map.mark_location(self.x, self.y, 1)

    def determine_turn(self):
        #TO be filled
        #IR Clearance to be around 8 at minimum.
        return Turn.Left

    #Is the robot near its destination?
    def is_near_destination(self):
        #Check if the robot is within a size 10(note size = 5 * 2) square of the objective.
        return self.map.is_point_in_sqr_radius(self.dest[0], self.dest[1], self.x, self.y, 20)

    def sweep(self, increment = 5, coneAngle = 50):
        #Turn left 90 degrees. Then start the sweep.
        self.turn_robot(-coneAngle/2)
        if (not self.QRFound):
            self.QR_scan()
       
        distance = 0
        angle = 0
        skip_angle = 20
        skip_set = 0
        while 1:
            #We hit the maximum angle.
            if angle >= coneAngle:
                break
            
            distance = Ultrasonic.read() + 6
            #if (not self.QRFound):
            #    image = thing.getimage((1920, 1088))
            #    QR_dist = thing.qrcodefunction(image)
            #    #For testing please remove later.
            #    self.move_forward(QR_dist - 10)
            #    if (QR_dist):
            #        self.QRFound = True
            
            index = int(angle/increment)
            if (distance <= 40):
                if (skip_set == 0):
                    self.obstacles.append( self.map.mark_relative_location(self.x, self.y, distance, self.orientation, 2))
                    self.turn_robot(5)
                    angle = angle +5
                else:
                    skip_set = 0
                    error = self.map.cone_error(self.x, self.y,self.orientation,skip_angle-increment)
                    for x in error:
                        if x in self.obstacles:
                            self.obstacles.remove(x)

                    self.turn_robot(skip_angle-increment)
                    angle = angle + skip_angle-increment


                #1.6 seems to be how much extra angle is needed to get correct angle
            else :
                error = self.map.cone_error(self.x, self.y,self.orientation,skip_angle)
                for x in error:
                    if x in self.obstacles:
                        self.obstacles.remove(x)
                self.turn_robot(5)
               
                angle = angle + increment
                skip_set = 1

            #R.map.display_map()
            time.sleep(.5)
        if (not self.QRFound):
            self.QR_scan()

    #Sweep using the left IR sensor.
    def ir_sweep(self, coneAngle, increment):
        offsetAngle = 90 - coneAngle/2
        Motors.turnDegrees(offsetAngle, 4)
        time.sleep(5)

        #Update position and mark robot position on map.
        self.update_position(0, self.orientation + offsetAngle)
        R.map.mark_robot_pos(self.x , self.y, self.orientation)

        measurements = []
        for angle in range(0, coneAngle, increment):
            IR_reading = IR.readLeft() + 2
            if (IR_reading < 100):
                print(IR_reading)
                measurements.append(IR_reading)
                self.map.mark_relative_location(self.x, self.y, IR_reading, self.orientation, 2)
                Motors.turnDegrees(increment, 4)
                time.sleep(0.4)
        
        return measurements

    def get_dest_angle(self):
        if (self.dest):
            return self.map.get_dist_angle(self.x, self.y, self.dest[0], self.dest[1])[1]

        else:
            return None

    def ultrasound_sweep(self, coneAngle, increment):
        #Turn left.
        self.turn_robot(-coneAngle/2)

        #Update position and mark robot position on map.
        measurements = []
        for angle in range(0, coneAngle, increment):
            US_reading = Ultrasonic.read() + 2
            if (US_reading < 100):
                print(US_reading)
                measurements.append(US_reading)
                #Turn robot at our specified angle slices.
            self.turn_robot(increment)
                
        self.turn_robot(-coneAngle/2)

        return angle

        #if (leftReading > 10):
        #    self.turn = Turn.Left
        #    self.turn_robot(-90)
        #    self.move_forward(leftReading/2)
        
        #elif (rightReading > 10):
        #    self.turn = Turn.Right
        #    self.turn_robot(90)
        #    self.move_forward(rightReading/2)
        
        #else:
        #    #We might wanna reverse.
        #    self.move_forward(-10)

    


    #Continuous move.
    def cont_move(self):
        #Face objective and sweep.
        self.face_objective()
        self.sweep()
        if (self.state == State.Returning):
            print("Now returning")
            self.dest = [self.map.x_length/2, 20]
        
        #If we find the QR and we are within some range, we can ram.
        if (self.QRFound and self.is_near_destination()):
            #ATTACK
            print("Attacking objective...")
            self.face_objective()
            self.move_forward(22)
            self.state = State.Returning
        
        
        D = R.check_collision()
        #Obstacle encountered.
        if D:
            self.move_forward(D-5)
            self.avoid_obstacle()       

        else:
            self.move_forward(25)

    def face_objective(self):
        if (not self.dest):
            print("No destination has been set!")
            return
        self.turn_robot(self.orientation-self.get_dest_angle())


    def QR_scan(self):
        code = thing.qrcodefunction(res)
        if code:
            # self.map.mark_location(self.dest[0],self.dest[1],4)
            self.move_cam_to_mid(code)
            self.QRFound = True #We found QR.

    def move_to_objective(self):
        return 0

    def check_collision(self):
        
        ori_rad = self.orientation * m.pi/180
        X = self.x + 7*m.sqrt(2)*m.cos(ori_rad+ m.atan(10/7))
        Y = self.y + 7*m.sqrt(2)*m.sin(ori_rad+m.atan(10/7))

        for a in range(30):
            for b in range(20):
                if (int(X+a*m.cos(ori_rad)+b*m.cos(ori_rad-m.pi/2)),int(Y+a*m.sin(ori_rad)+b*m.sin(ori_rad-m.pi/2))) in self.obstacles:
                    return a 
        return None
                

R = myRobot(Turn.Left)

if (run):
    
    rc = RoverController()
    rc.setReadTimeout(10)
    rc.connectIP()
    
    while 1:
##        R.QR_scan()
##        time.sleep(3)
        R.cont_move()
