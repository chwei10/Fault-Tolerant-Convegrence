import smallestenclosingcircle
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML, display

GATHERING = False

class Robot:
    def __init__(self, id, observation_radius, speed, x, y, disable=False, trajectory_x=None, trajectory_y=None, terminate=False):
        self.id = id
        self.observation_radius = observation_radius
        self.speed = speed
        self.x = x
        self.y = y
        self.center_x = None  
        self.center_y = None  
        self.same_center = 0
        self.trajectory_x = trajectory_x or []
        self.trajectory_y = trajectory_y or []
        self.disable = disable
        self.terminate = terminate
        # Add the first pair of coordinates to the trajectory
        self.trajectory_x.append(x)
        self.trajectory_y.append(y)

    def get_distance(self, other_robot):
        dx = self.x - other_robot.x
        dy = self.y - other_robot.y
        distance = (dx**2 + dy**2)**0.5
        return distance
    
    def calculate_cosine_sine(point1, self, point3):
        x1, y1 = point1
        x2, y2 = self
        x3, y3 = point3
        
        # Calculate the distances between the points, point2 is the origin
        dx1 = x2 - x1
        dy1 = y2 - y1
        dx2 = x3 - x2
        dy2 = y3 - y2
        
        # Calculate the dot product of the vectors
        dot_product = dx1 * dx2 + dy1 * dy2
        
        # Calculate the magnitudes of the vectors
        magnitude1 = math.sqrt(dx1 ** 2 + dy1 ** 2)
        magnitude2 = math.sqrt(dx2 ** 2 + dy2 ** 2)
        
        # Calculate the cosine value
        cosine = dot_product / (magnitude1 * magnitude2)
        
        # Calculate the sine value
        sine = math.sqrt(1 - cosine ** 2)
        
        return cosine, sine
    
    def is_in_observation_range(self, robot):
        distance = self.get_distance(robot)
        return distance <= self.observation_radius
    
    def observe(self, robots):
        return [robot for robot in robots if (self.id != robot.id and self.is_in_observation_range(robot))]
        
    def trigonometrics(self, c, j):
        a = np.array(c)
        b = np.array((self.x,self.y))
        c = np.array(j)
        ba = a - b
        bc = c - b
        if (ba_value := np.linalg.norm(ba)) == 0 or (bc_value := np.linalg.norm(bc)) == 0: # Special Case: same x/y coordinates, theta = 0
            return 1, 0
        cosine = np.dot(ba, bc) / (ba_value * bc_value)
        # Special Case, Python's limited computing capacity on float, cos(theta) slightly greater than 1 (e.g., 1.00000001), then let it be 1
        if cosine > 1: 
            cosine = 1
        sq_sine = (1 - cosine**2)
        sine = math.sqrt( 0 if sq_sine < 0 else sq_sine)
        return cosine, sine

    # Use smallest enclosing circle to find the center of the robots and move towards the center
    def move(self, robots_visible, robots):
        # if self.disable:
        #     # Disabled robots don't move, update the trajectory with current position
        #     self.trajectory_x.append(self.x)
        #     self.trajectory_y.append(self.y)
        #     return
        
        if len(robots_visible) == len(robots) - 1:
            GATHERING = True
            for robot in robots_visible:
                if(self.get_distance(robot) > 0.1):
                    GATHERING = False
                    print("Not gathering!")
                    break
            if GATHERING:
                self.terminate = True
                print("Gathering!")
                return            


        if len(robots_visible) == 0:
            return

        # Find the center of the robots
        current_center_x, current_center_y, _ = smallestenclosingcircle.make_circle([(robot.x, robot.y) for robot in robots_visible])

        # Check if the center has moved or not
        if self.center_x is not None and self.disable is False:
            dist = math.sqrt((self.center_x - current_center_x) ** 2 + (self.center_y - current_center_y) ** 2)
            if dist == 0:
                self.same_center += 1
            else:
                self.same_center = 0
            if self.same_center == 5:
                print("Robot", self.id, "has been stable. Terminating...")
                self.terminate = True
                return

        self.center_x = current_center_x
        self.center_y = current_center_y
        # print("Center: ", center_x, center_y)
        # For each robot, calculate the distance of Lj and choose the minimum one as LIMIT
        Lj_list = []
        for robot in robots_visible:
            cosine, sine = self.trigonometrics((self.center_x, self.center_y), (robot.x, robot.y))
            Lj = ((math.dist((self.x, self.y), (robot.x, robot.y))/2 * cosine) 
                           + ((self.observation_radius/2)**2 - (math.dist((self.x,self.y), (robot.x, robot.y))/2 * sine) ** 2) ** 0.5 )
            Lj_list.append(Lj)

        # Find the minimum based on the real part of complex numbers
        LIMIT = min(Lj_list)  
        GOAL = math.dist((self.x, self.y), (self.center_x, self.center_y))
        MOVE = min(LIMIT, GOAL, self.speed)

        if  self.disable:
            # it does not move
            destination_x = self.x
            destination_y = self.y
        elif GOAL == 0:
            # When there are two faulted robots, the center is the faulted robot, so the robot will not move
            # self.terminate = True
            return
        else:
            destination_x = self.x + MOVE * (self.center_x - self.x) / math.dist((self.x, self.y), (self.center_x, self.center_y))
            destination_y = self.y + MOVE * (self.center_y - self.y) / math.dist((self.x, self.y), (self.center_x, self.center_y))
        # Divide the track into frames for trajectory animation
        for i in range(1, 11):
            self.trajectory_x.append(self.x + i * (destination_x - self.x) / 10)
            self.trajectory_y.append(self.y + i * (destination_y - self.y) / 10)

        self.x = destination_x
        self.y = destination_y
