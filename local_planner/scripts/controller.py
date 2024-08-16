import math
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class Controller():
    def __init__(self):

        self.Kp_vel = 2
        self.Kp_theta = 6

        self.vel_limit = 0.6
        self.w_limit  = 0.8

        self.error_x = 0.0
        self.error_y = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0
        self.yaw = 0.0

    def get_cmdVel(self,odom_msg: Odometry, target_x, target_y):

        
        
        return (vel_x,vel_angular)
    
    def goal_reached(self):
        if abs(self.error_x) < 0.05 and abs(self.error_y) < 0.05:
            return True
        else:
            return False
        
    def waypoint_reached(self):
        if abs(self.error_x) < 0.08 and abs(self.error_y) < 0.08:
            return True
        else:
            return False
        


