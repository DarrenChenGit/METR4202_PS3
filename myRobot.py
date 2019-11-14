import math as m
import numpy
import enum
import time
import thing
from map import Map
from micromelon import *

run = 1
#res = (1920,1080)
res = (1280,720)
#res = (640,480)

delta = 200
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
    def __init__(self, turn, x_size = 200, y_size = 200, speed = 2, turnSpeed = 6, orientation = 90):
        self.x = int(x_size/2)
        self.y = int(20)
        self.orientation = orientation #Orientation w.r.t to world coordinates.
        self.speed = speed #How fast robot move forward/backward
        self.turnSpeed = turnSpeed #How fast robot should turn.
        self.turn = turn
        self.map = Map(x_size, y_size)
        self.map.mark_robot_pos(self.x,self.y,orientation)
        self.state = State.Explore
        self.dest = [100,195]
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
        if abs(degrees) > 180:
            degrees = (degrees - 360) * (degrees/abs(degrees))
            if abs(degrees) >= 160:
                for x in range (0,180,10):
                    self.turn_robot(10)
                self.face_objective()
                return None
            
        delay = ((2 * m.pi * 8) * (abs(degrees)/360)) / self.turnSpeed 
        self.orientation -= degrees
        if run:
            if abs(degrees) <= 10:
                degrees = degrees * 1.1

            if  45 <= int(abs(degrees)) <= 90:
                degrees = degrees * .95

            #Slower turn because tracks may come off.
            
                
                
            Motors.turnDegrees(int(degrees), self.turnSpeed)
            time.sleep(0.7 + delay)
            

        #We can calibrate the stuff.
        #Mark the robot position.
        self.map.mark_robot_pos(self.x , self.y, self.orientation)

    def move_forward(self, distance):
        ##Calculate where the robot should be if it moved.
        # destX = self.x + distance * m.cos(m.radians(self.orientation))
        # destY = self.y + distance * m.sin(m.radians(self.orientation))
        Motors.moveDistance(distance)
        time.sleep(abs(distance/15))
        self.update_position(distance, self.orientation)
        self.map.mark_robot_pos(self.x , self.y, self.orientation)

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
        
        #Keep moving forward until IR sensor is > 25
        while Ultrasonic.read() > 5:
            if self.IR_read(sensor) > 10:
                a = 0
                while Ultrasonic.read() > 5:
                    self.move_forward(2)
                    a = a + 2
                    if a >= 15:
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
    # results = [distance, dif, QR_Center, min, max]
        dif = results[1]
        qrcentre = results[2]
        angleToTurn = dif / 29 #Estimation
        self.face_objective()
        #While the distance not = 0, when we cant find QR the distance returned by function is 0.
        # while results[0] != 0:
        #     results = thing.qrcodefunction(res)
        #     time.sleep(2)
        #     qrcentre = int(results[2])

            
        #     if qrcentre > (mid+delta):
        #         self.turn_robot(5)
                

        #     elif qrcentre < (mid-delta):
        #         self.turn_robot(-5)

        #     else:
        #         x = self.x + results[0] * m.cos(m.radians(self.orientation))
        #         y = self.y + results[0] * m.sin(m.radians(self.orientation))
        #         self.dest = [int(round(x)), int(round(y))]
        #         print("Dest is ", int(x), int(y))
        #         self.QRFound = True #We found QR.
        #         break

#MAPPING FUNCTIONS
    def update_position(self, distance, orientation):
        angle_rad = m.radians(orientation)
        x = distance * m.cos(angle_rad)
        y = distance * m.sin(angle_rad)
        self.x += x
        self.y += y
        self.orientation = orientation
        self.map.mark_location(self.x, self.y, 1)

    #Is the robot near its destination?
    def is_near_destination(self):
        #Check if the robot is within a size 10(note size = 5 * 2) square of the objective.
        return self.map.is_point_in_sqr_radius(self.dest[0], self.dest[1], self.x, self.y, int(60))

    def sweep(self, coneAngle = 50, increment = 5):
        #Turn left 90 degrees. Then start the sweep.
        self.turn_robot(int(-coneAngle/2))
        if (not self.QRFound):
            self.QR_scan()
        
       
        distance = 0
        angle = 0
        skip_angle = 20
        skip_set = 0
        while 1:
            #We hit the maximum angle.
            if angle >= coneAngle:
                #If we found QR at the right edge of cone.
            
                #Break only if we found it.
                if (not self.QRFound):
                    self.QR_scan()
                self.turn_robot(-coneAngle/2)
                break
            #Read ultrasound.
            distance = Ultrasonic.read() + 6

            #Look for QR constantly.
            # if (self.state != State.Returning):
            #     self.QR_scan()
            if angle == coneAngle/2:
                self.QR_scan()    
            if (distance <= 40):
                if (skip_set == 0):
                    self.obstacles.append(self.map.mark_relative_location(self.x, self.y, distance, self.orientation, 2))
                    self.turn_robot(increment)
                    angle = angle + increment
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
                self.turn_robot(increment)
               
                angle = angle + increment
                skip_set = 1

            #R.map.display_map()

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

    def get_dest_distance(self):
        if (self.dest):
            return self.map.get_dist_angle(self.x, self.y, self.dest[0], self.dest[1])[0]

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

    #Continuous move. Returns true if robot returned home.
    def cont_move(self):
        #Face objective and sweep.
        self.face_objective()
        self.sweep(60, 10)
        self.face_objective()
        

        #Returning after yeeting the QR flag.
        if (self.state == State.Returning):
            print("Now returning")
            self.dest = [self.map.x_length/2, 20]
            #If we are near, park the robot.
            if (self.is_near_destination()):
                self.face_objective()
                distance = self.get_dest_distance()
                self.move_forward(distance)
                return True
                
        
        #If we find the QR and we are within some range, we can ram.
        elif (self.is_near_destination() and self.QRFound):
         
            #ATTACK
            print("Attacking objective...")
            self.face_objective()
            self.move_forward(self.get_dest_distance() - 7)
            self.state = State.Returning
            return False
        
        
        D = R.check_collision()
        #Obstacle encountered.
        if D:
            self.move_forward(D-5)
            self.avoid_obstacle()       

        else:
            self.move_forward(30)

        return False

    def face_objective(self):
        if (not self.dest):
            print("No destination has been set!")
            return
        self.turn_robot(self.orientation-self.get_dest_angle())


    #Scan for a QR code and try to align to it. Returns true if found,
    #else returns false for no QR.
    def QR_scan(self):
        code = thing.qrcodefunction(res)
        if code[0] != 0:
            # self.map.mark_location(self.dest[0],self.dest[1],4)
            dif = code[1]
            angleToTurn = (dif+49) / 22.5 #Estimation
            print("Angle to turn:", angleToTurn)
            x = self.x + code[0] * m.cos(m.radians(self.orientation - angleToTurn))
            y = self.y + code[0] * m.sin(m.radians(self.orientation - angleToTurn))
            print("Dest set to", x, y)
            self.dest = [x,y]
            self.QRFound = True
            #self.move_cam_to_mid(code)
            return True

        return False

    def check_collision(self):
        
        ori_rad = self.orientation * m.pi/180
        X = self.x + 7*m.sqrt(2)*m.cos(ori_rad+ m.atan(10/7))
        Y = self.y + 7*m.sqrt(2)*m.sin(ori_rad+m.atan(10/7))

        for a in range(30):
            for b in range(20):
                if (int(X+a*m.cos(ori_rad)+b*m.cos(ori_rad-m.pi/2)), int(Y+ a*m.sin(ori_rad) + b*m.sin(ori_rad-m.pi/2))) in self.obstacles:
                    return a 
        return None
                

R = myRobot(Turn.Left)

if (run):
    
    rc = RoverController()
    #rc.setReadTimeout(10)
    rc.connectIP()
    
    
    while 1:
        if (R.cont_move()):
            rc.disarm
            break
            print("All done")
        time.sleep(1)
        
