import sys
from math import pi, cos, sin, sqrt, atan, acos
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
        self.lower_arm.length = 650
        self.upper_arm.length = 103


        # inverse config
        self.newton_error = 0.01

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
        print(lower_arm_angle, upper_arm_angle,x,y)
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

    def createIntermediatePoints(self, init_x, init_y, end_x, end_y, num_points):
        delta_x = (end_x - init_x)/(num_points - 1)
        delta_y = (end_y - init_y)/(num_points - 1)

        points = [None] * num_points
        for i in range(num_points):
            points[i] = [init_x+delta_x*i, init_y+delta_y*i]
            
        return points

    def euclideanDistance(self, init_x, init_y, end_x, end_y):
        return sqrt((end_x - init_x) ** 2 + (end_y - init_y) ** 2)
    
    def moveToPos(self, end_x, end_y):
        init_x, init_y = self.getPosition()

        points = self.createIntermediatePoints(init_x, init_y, end_x, end_y, int(self.euclideanDistance(init_x, init_y, end_x, end_y) // 10))

        # print(init_x,init_y, points)

    def moveToMid(self):
        pass

    def analyticalApproach(self, x,y):
        # D = cos(theta2)
        D_cos = (x**2 + y**2 - self.lower_arm.length**2 - self.upper_arm.length**2)/ (2*(self.lower_arm.length* self.upper_arm.length))
        # print(D_cos)
        theta_2 = acos(D_cos)
        theta_1 = atan(y/x) - atan(self.upper_arm.length*sin(theta_2)/(self.lower_arm.length + self.upper_arm.length*cos(theta_2)))
        print(f"theta1:", theta_1, "theta_2:", theta_2)
        return theta_1, theta_2
        

if __name__ == "__main__":
    arm = Arm()
    arm.moveToPos(10,20)
    arm.analyticalApproach(750,0)
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
