#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from local_planner_interfaces.msg import Waypoint
from tf_transformations import euler_from_quaternion
import numpy as np

class IPCController(Node):
    def __init__(self):
        super().__init__('ipc_controller')
        # publishers and subscribers initialization
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.cmdVel_pub_ = self.create_publisher(Twist,'/cmd_vel',10)
        self.waypoint_subscriber = self.create_subscription(Waypoint,'/waypoints',self.waypoint_callback,10)

        self.cmdVel_timer = self.create_timer(0.1,self.cmdVel_callback)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.waypoint_x = 0.0
        self.waypoint_y = 0.0

        self.cnt = 0

        self.get_logger().info("IPC CONTROLLER RUNNING")

    def IPC_controller(self, waypoint_x, waypoint_y):
        target = np.array([waypoint_x, waypoint_y])
        pose = [self.robot_x, self.robot_y, self.robot_yaw]

        # Controller Implementation
        rel_bearing = pose[2] - np.arctan2(pose[1] - waypoint_y, pose[0] - waypoint_x)

        if rel_bearing >= np.pi:
            rel_bearing -= 2 * np.pi
        elif rel_bearing <= -np.pi:
            rel_bearing += 2 * np.pi

        if rel_bearing > np.pi / 2:
            sigma = rel_bearing - np.pi
        elif rel_bearing < -np.pi / 2:
            sigma = rel_bearing + np.pi
        else:
            sigma = rel_bearing

        K_1 = 0.2
        K_2 = 0.1

        distance = np.linalg.norm(pose[:2] - target)
        v = -K_1 * np.tanh(distance) * np.sign(np.cos(rel_bearing))
        omega = -K_2 * np.sign(sigma) * np.abs(sigma)**0.5 + (v / distance) * np.sin(rel_bearing)
        
        return v, omega
    
    def odom_callback(self, odom: Odometry):
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y

        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w

        self.robot_yaw = euler_from_quaternion([x,y,z,w])[2]
    
    def waypoint_callback(self, wp: Waypoint):
        self.waypoint_x = wp.waypoint_x
        self.waypoint_y = wp.waypoint_y
    
    def cmdVel_callback(self):
        vel = Twist()
        self.cnt += 1
        if self.cnt > 10:

            v, w = self.IPC_controller(self.waypoint_x, self.waypoint_y)
            vel.angular.z = w
            vel.linear.x = v

            self.cmdVel_pub_.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = IPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  
