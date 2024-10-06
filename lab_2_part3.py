import sys
from math import pi, cos, sin, sqrt, atan, acos, atan2 
import numpy as np
# from ev3dev2.motor import LargeMotor, SpeedPercent, OUTPUT_A, OUTPUT_B
class TempArm():
    pass

class TempMotor():
    lower_arm = TempArm()
    upper_arm = TempArm()

    def stop(self):
        pass

    def run_to_abs_pos(self):
        self.position = self.position_sp

    def wait_while(self, m):
        pass

class Arm():
    def __init__(self, speed = 15, stop_action = "coast"):
        # self.lower_arm = LargeMotor(OUTPUT_A)
        # self.upper_arm = LargeMotor(OUTPUT_B)
        self.lower_arm = TempMotor()
        self.upper_arm = TempMotor()
        self.lower_arm.speed_sp = speed
        self.lower_arm.stop_action = stop_action
        self.upper_arm.speed_sp = speed
        self.upper_arm.stop_action =stop_action

        # calibrated config
        self.lower_arm.lower_bound = 41316
        self.lower_arm.upper_bound = 41534
        self.lower_arm.midpoint = 41425

        self.upper_arm.lower_bound = 41316
        self.upper_arm.upper_bound = 41534
        self.upper_arm.midpoint = 41425

        # arm lengths in mm
        self.lower_arm.length = 106
        self.upper_arm.length = 103


        # inverse config
        self.newton_error = 0.01
        self.max_iter = 100

        # move to initial position
        self.moveArmsAbsolute(self.lower_arm.midpoint, self.upper_arm.midpoint)
    
    def __del__(self):
        self.stop()

    def getRadFromDeg(self, deg):
        return deg*pi/180

    def getDegFromRad(self, rad):
        return rad*180/pi

    def getAngleOfArm(self, arm, radians=False):
        angle = arm.position - arm.midpoint
        return self.getRadFromDeg(angle) if radians else angle
    
    # of end effector
    def getPosition(self):
        
        lower_arm_angle = self.getAngleOfArm(self.lower_arm, True)
        upper_arm_angle = self.getAngleOfArm(self.upper_arm, True)
            
        x = self.lower_arm.length * cos(lower_arm_angle) + self.upper_arm.length * cos(lower_arm_angle + upper_arm_angle)
        y = self.lower_arm.length * sin(lower_arm_angle) + self.upper_arm.length * sin(lower_arm_angle + upper_arm_angle)
        return x, y
    
    def getLowerArm(self):
        return self.lower_arm

    def getUpperArm(self):
        return self.upper_arm

    def stop(self):
        self.lower_arm.stop()
        self.upper_arm.stop()

    def moveArmsAbsolute(self, lower_pos, upper_pos):
        self.lower_arm.position_sp = lower_pos
        self.upper_arm.position_sp = upper_pos
        self.lower_arm.run_to_abs_pos()
        self.upper_arm.run_to_abs_pos()
        self.lower_arm.wait_while("running")
        self.upper_arm.wait_while("running")

    def euclideanDistance(self, init_x, init_y, end_x, end_y):
        return sqrt((end_x - init_x) ** 2 + (end_y - init_y) ** 2)
    
    def moveToPos(self, type_arg, end_x, end_y):
        init_x, init_y = self.getPosition()
        print(self.getPosition())

        points = self.createIntermediatePoints(init_x, init_y, end_x, end_y, max(1, int(self.euclideanDistance(init_x, init_y, end_x, end_y) // 10)))
        if type_arg == "anal":
            angles = self.analyticalSolve(points)

        for theta_1, theta_2 in angles:
            self.moveArmsAbsolute(self.getAbsPosFromTheta(self.lower_arm, theta_1), self.getAbsPosFromTheta(self.upper_arm, theta_2))
        print(self.getPosition())

    def moveToMid(self, type_arg):
        pass
    
    def analyticalSolve(self, points):
        curr_theta_2 = self.getAngleOfArm(self.upper_arm, True)
        for i, (x, y) in enumerate(points):
            d = (x ** 2 + y ** 2 - self.lower_arm.length ** 2 - self.upper_arm.length ** 2)/(2*self.lower_arm.length*self.upper_arm.length)
            # theta_2 = atan(sqrt(1-d ** 2) / d)
            theta_2 = acos(d)
            if (abs(curr_theta_2 - theta_2) < abs(curr_theta_2 + theta_2)):
                curr_theta_2 = theta_2
            else:
                curr_theta_2 = -theta_2
            
            points[i] = [atan(y/x) - atan((self.upper_arm.length*sin(curr_theta_2))/(self.lower_arm.length + self.upper_arm.length*cos(curr_theta_2))), curr_theta_2]
        
        return points
    
    def createIntermediatePoints(self, init_x, init_y, end_x, end_y, num_points):
        delta_x = (end_x - init_x)/(num_points)
        delta_y = (end_y - init_y)/(num_points)

        points = [None] * num_points
        for i in range(1, num_points + 1):
            points[i - 1] = [init_x+delta_x*i, init_y+delta_y*i]
            
        return points
    
    # of end effector
    def normalize_angle(self,angle):
        while angle < -2 * np.pi:
            angle += 4 * np.pi
        while angle > 2 * np.pi:
            angle -= 4 * np.pi
        return angle

    def getPositioWithKnownAngles(self, theta_1, theta_2):
        theta_1 = self.normalize_angle(theta_1)
        theta_2 = self.normalize_angle(theta_2)

        x = self.lower_arm.length * cos(theta_1) + self.upper_arm.length * cos(theta_1 + theta_2)
        y = self.lower_arm.length * sin(theta_1) + self.upper_arm.length * sin(theta_1 + theta_2)
        # print(lower_arm_angle, upper_arm_angle,x,y)
        return x, y

   
    
    def velocity_kinematics(self, theta_1, theta_2):
        j_22 = self.upper_arm.length * cos(theta_1 + theta_2)
        j_21 = self.upper_arm.length * sin(theta_1 + theta_2)
        j_12 =  self.lower_arm.length * cos(theta_1) + self.upper_arm.length * cos(theta_1 + theta_2)
        j_11 = - self.lower_arm.length * sin(theta_1) - self.upper_arm.length * sin(theta_1 + theta_2)
        determinant = self.lower_arm.length * self.upper_arm.length * sin(theta_2)
        

        if (sin(theta_2) == 0):
          ## what to do when singular configuration met, use a super small value ex: 1e-6
            determinant = 1e-6
            # print("here")
            
        determinant_inverse = 1/determinant
        #perform matrix to get jacobian
        vel_kin = [
            [determinant_inverse * j_11, determinant_inverse * j_12],
            [determinant_inverse * j_21, determinant_inverse * j_22]
            ]

        return vel_kin

    def newtonApproach(self, target_x, target_y):
        theta_1 = self.getAngleOfArm(self.lower_arm)
        theta_2 = self.getAngleOfArm(self.upper_arm)
    
        angles = [[0, 0] for i in range(self.max_iter)] #initialize empty arrays

        for i in range(0, self.max_iter): ## or should we be slowly incrementing the x,y till we get to the desired location
            
            curr_x, curr_y = self.getPositioWithKnownAngles(theta_1, theta_2) # forward kinematics 
            print(f"round", i, "current location x", curr_x, ", current location y", curr_y)
            error_position = [[float(target_x - curr_x)],[float(target_y - curr_y)]]
            
            delta_pos = self.euclideanDistance(curr_x, curr_y, target_x, target_y)

            if(delta_pos < self.newton_error):
                break

            vel_kin = self.velocity_kinematics(theta_1, theta_2)
            # print(f"error of the position:", error_position, "distance to end point", delta_pos)
            
            delta_theta = [
                vel_kin[0][0] * error_position[0][0] + vel_kin[0][1] * error_position[1][0],
                vel_kin[1][0] * error_position[0][0] + vel_kin[1][1] * error_position[1][0]
            ]

            # target_angle = atan2(error_position[0][0], error_position[1][0])
            # current_angle = atan2(curr_y, curr_x)

            # print("target angle:",target_angle, "current_angle", current_angle)
            
            max_change = 1

            if (delta_theta[0] > max_change):
                delta_theta[0] = max_change
            elif (delta_theta[0] < -max_change):
                delta_theta[0] = -max_change
                

            if (delta_theta[1] > max_change):
                delta_theta[1] = max_change
            elif (delta_theta[1] < -max_change):
                delta_theta[1] = -max_change


            theta_1 += delta_theta[0]
            theta_2 += delta_theta[1]
            

            angles[i][0] = theta_1
            angles[i][1] = theta_2
            print(f"angles:", theta_1, ',', theta_2, ",")
        return angles



        

if __name__ == "__main__":
    arm = Arm()
    # arm.moveToPos(10,20)
    # arm.analyticalApproach(750,0)
    init_x, init_y = arm.getPosition()
    end_x = 0
    end_y = 209
    points = arm.createIntermediatePoints(init_x, init_y, end_x, end_y, max(1, int(arm.euclideanDistance(init_x, init_y, end_x, end_y) // 10)))
    arm.newtonApproach(30, 100)
    
    # if len(sys.argv) != 2:
    #     print("Error: Exactly one argument (pos/mid) is required.")
    #     sys.exit(1)
    
    # input_arg = sys.argv[1]

    # if input_arg not in ["pos", "mid"]:
    #     print("Error: Argument must be 'pos' or 'mid'.")
    #     sys.exit(1)

    # if input_arg == "pos":
    #     poses = input("enter space separated desired x and y: ")
    #     x, y = map(int, poses.split())
    #     arm.moveToPos(x, y)
    # else:
    #     arm.moveToMid()
