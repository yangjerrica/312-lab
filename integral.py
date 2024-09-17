from time import sleep
from math import cos, sin

class Robot:
    def __init__(self, integration_steps, max_rot_sec, d, radius):
        self.integration_steps = integration_steps
        self.max_rot_sec = max_rot_sec
        self.d = d
        self.radius = radius

    def findDistance(self, velocity, angular_velocity, seconds, prev_x, prev_y, prev_orientation):
        x = prev_x
        y = prev_y
        orientation_1 = 0
        orientation_2 = 0
        t_1 = 0
        t_2 = 0
        delta_angular_velocity = angular_velocity / self.integration_steps
        delta_sec = seconds / self.integration_steps
        
        for i in range(self.integration_steps):
            if(angular_velocity):
                orientation_1 = prev_orientation + delta_angular_velocity * i
                orientation_2 = prev_orientation + delta_angular_velocity * (i+1)
            else:
                orientation_1 = prev_orientation
                orientation_2 = prev_orientation
            t_2 += delta_sec
            x += t_2 * velocity * cos(orientation_2) - t_1 * velocity * cos(orientation_1)
            y += t_2 * velocity * sin(orientation_2) - t_1 * velocity * sin(orientation_1)
            
            print(f"x: {x}, y: {y}, orientation: {orientation_2}")

        return x, y, orientation_2

    def logPosition(self, speed_left, speed_right, seconds, prev_x, prev_y, prev_orientation):
        # find velocities of each wheel
        v_left = speed_left / 100 * self.radius * self.max_rot_sec
        v_right = speed_right / 100 * self.radius * self.max_rot_sec

        print(f"v_left: {v_left}")

        # turning on angle
        if v_left != v_right:
            angular_velocity = (v_right - v_left) / (2 * self.d)
            rotation_radius = self.d * (v_right + v_left) / (v_right - v_left)

            velocity = rotation_radius * angular_velocity
            print(f"velocity: {velocity}, rotation radius: {rotation_radius}")
        # straight line
        else:
            velocity = v_left
            angular_velocity = 0

        distance_x, distance_y, orientation = self.findDistance(
            velocity, angular_velocity, seconds, prev_x, prev_y, prev_orientation
        )

        print(f"Distance X: {distance_x}, Distance Y: {distance_y}, Orientation: {orientation}")

        return distance_x, distance_y, orientation

    def go(self, speed_left, speed_right, seconds):
        print(f"Going: Left Speed {speed_left}, Right Speed {speed_right}, for {seconds} seconds")
        sleep(seconds)  # Simulate motor movement

    def moveDeadReckoning(self, commands):
        prev_x = 0
        prev_y = 0
        prev_orientation = 0
        for command in commands:
            self.go(*command)
            distance_x, distance_y, orientation = self.logPosition(*command, prev_x, prev_y, prev_orientation)
            prev_x = distance_x
            prev_y = distance_y
            prev_orientation = orientation


commands = [
    [80, 60, 2],
    [60, 60, 1],
    [-50, 80, 2],
]

robot = Robot(integration_steps=10, max_rot_sec=2.13, d=9.75, radius = 34.4)
robot.moveDeadReckoning(commands)

