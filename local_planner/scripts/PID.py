import math
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class PIDController():
    def __init__(self):

        self.Kp_vel = 2
        self.Kp_theta = 6

        self.vel_limit = 0.15
        self.w_limit  = 0.3

        self.error_x = 0.0
        self.error_y = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0
        self.yaw = 0.0

    def get_cmdVel(self,odom_msg: Odometry, target_x, target_y):

        self.error_x = target_x - odom_msg.pose.pose.position.x
        self.error_y = target_y - odom_msg.pose.pose.position.y

        error_position = math.sqrt(self.error_x**2 + self.error_y**2)

        self.w = odom_msg.pose.pose.orientation.w
        self.x = odom_msg.pose.pose.orientation.x
        self.y = odom_msg.pose.pose.orientation.y
        self.z = odom_msg.pose.pose.orientation.z

        self.yaw = euler_from_quaternion([self.x,self.y,self.z,self.w])[2]

        if abs(self.error_x) < 0.001:
            self.error_x = 0.001

        target_theta  = math.atan2(self.error_y,self.error_x)
        # self.target_theta  = math.atan(self.self.error_y/self.self.error_x)
        error_theta = target_theta - self.yaw

        if -math.pi < error_theta < -math.pi/4 or math.pi > error_theta > math.pi/4:
            vel_x = 0.0
            vel_angular = self.Kp_theta * error_theta
        else:
            vel_x = self.Kp_vel * error_position
            vel_angular = self.Kp_theta * error_theta

        vel_x = min(vel_x, self.vel_limit)
        vel_x = max(vel_x, -self.vel_limit)

        vel_angular = min(vel_angular, self.w_limit)
        vel_angular = max(vel_angular, -self.w_limit)
        # self.get_logger().info("getting odom!")

        if self.goal_reached():
            return (0.0,0.0)
        
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
        

