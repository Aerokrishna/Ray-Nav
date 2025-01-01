import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion

class PIDControllerHolonomic():
    def __init__(self):
        
        # returns vx vy vw
        self.Kp_vel = 1.0
        self.Kp_theta = 0

        self.vel_limit = 0.3
        self.w_limit  = 0.1

        self.error_x = 0.0
        self.error_y = 0.0
        self.error_theta = 0.0

    def get_drone_cmdVel(self,odom_msg: Pose, target_x, target_y):

        self.error_x = target_x - odom_msg.position.x
        self.error_y = target_y - odom_msg.position.y

        w = odom_msg.orientation.w
        x = odom_msg.orientation.x
        y = odom_msg.orientation.y
        z = odom_msg.orientation.z

        yaw = euler_from_quaternion([x,y,z,w])[2]

        if abs(self.error_x) < 0.001 or abs(self.error_y) < 0.001:
            self.error_x = 0.001
            self.error_y = 0.001

        # error_theta = target_theta - yaw

        global_vel_x = self.Kp_vel * self.error_x
        global_vel_y = self.Kp_vel * self.error_y
        vel_angular = 0.0

        # vel_angular = min(vel_angular, self.w_limit)
        # vel_angular = max(vel_angular, -self.w_limit)
        # self.get_logger().info("getting odom!")

        # compute velocity in the global frame
        # give velocity to the robot in local frame
        # print(yaw)
        local_vel_x = global_vel_x * np.cos(yaw) + global_vel_y * np.sin(yaw)
        local_vel_y = global_vel_y * np.cos(yaw) - global_vel_x * np.sin(yaw)

        local_vel_x = min(local_vel_x, self.vel_limit)
        local_vel_x = max(local_vel_x, -self.vel_limit)

        local_vel_y = min(local_vel_y, self.vel_limit)
        local_vel_y = max(local_vel_y, -self.vel_limit)


        if self.goal_reached():
            return (0.0,0.0,0.0)
        
        return (local_vel_x,local_vel_y,vel_angular)

    
    def get_cmdVel(self,odom_msg: Odometry, target_x, target_y):

        self.error_x = target_x - odom_msg.pose.pose.position.x
        self.error_y = target_y - odom_msg.pose.pose.position.y
        print(self.error_x, self.error_y)
        
        w = odom_msg.pose.pose.orientation.w
        x = odom_msg.pose.pose.orientation.x
        y = odom_msg.pose.pose.orientation.y
        z = odom_msg.pose.pose.orientation.z

        yaw = euler_from_quaternion([x,y,z,w])[2]

        if abs(self.error_x) < 0.001 or abs(self.error_y) < 0.001:
            self.error_x = 0.001
            self.error_y = 0.001

        # error_theta = target_theta - yaw

        global_vel_x = self.Kp_vel * self.error_x
        global_vel_y = self.Kp_vel * self.error_y
        vel_angular = 0.0

        # vel_angular = min(vel_angular, self.w_limit)
        # vel_angular = max(vel_angular, -self.w_limit)
        # self.get_logger().info("getting odom!")

        # compute velocity in the global frame
        # give velocity to the robot in local frame
        local_vel_x = global_vel_x * np.cos(yaw) + global_vel_y * np.sin(yaw)
        local_vel_y = global_vel_y * np.cos(yaw) - global_vel_x * np.sin(yaw)

        local_vel_x = min(local_vel_x, self.vel_limit)
        local_vel_x = max(local_vel_x, -self.vel_limit)

        local_vel_y = min(local_vel_y, self.vel_limit)
        local_vel_y = max(local_vel_y, -self.vel_limit)

        if self.goal_reached():
            return (0.0,0.0,0.0)
        
        return (local_vel_x,local_vel_y,vel_angular)
    
    def goal_reached(self):
        if abs(self.error_x) < 0.1 and abs(self.error_y) < 0.1:

            print('GOAL REACHED!')
            return True
        else:
            return False
        
    def waypoint_reached(self):
        if abs(self.error_x) < 0.08 and abs(self.error_y) < 0.08:
            return True
        else:
            return False
        

