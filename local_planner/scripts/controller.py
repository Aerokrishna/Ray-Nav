#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from PID_holonomic import PIDControllerHolonomic
from local_planner_interfaces.msg import Waypoint
from tf_transformations import euler_from_quaternion
import numpy as np

class IPCController(Node):
    def __init__(self):
        super().__init__('ipc_controller')
        # publishers and subscribers initialization
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.drone_pose_sub = self.create_subscription(Pose,'/drone/gt_pose',self.drone_pose_callback,10)

        self.cmdVel_pub_ = self.create_publisher(Twist,'/drone/cmd_vel',10)
        self.waypoint_subscriber = self.create_subscription(Waypoint,'/waypoints',self.waypoint_callback,10)

        self.cmdVel_timer = self.create_timer(0.1,self.cmdVel_callback)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.waypoint_x = 1.0
        self.waypoint_y = 1.0
        # self.controller = PIDController()
        self.controller = PIDControllerHolonomic()

        self.cnt = 0
        self.odom_msg = Odometry()
        self.drone_pose = Pose()

        self.get_logger().info("PID CONTROLLER RUNNING")
    
    def odom_callback(self, odom: Odometry):
        self.odom_msg = odom

    def drone_pose_callback(self, drone_pose : Pose):
        self.drone_pose = drone_pose

    def waypoint_callback(self, wp: Waypoint):
        self.waypoint_x = wp.waypoint_x
        self.waypoint_y = wp.waypoint_y
    
    def cmdVel_callback(self):
        vel = Twist()
        self.cnt += 1
        if self.cnt > 10:

            # v, w = self.controller.get_cmdVel(self.odom_msg, self.waypoint_x, self.waypoint_y)
            # vel.angular.z = w
            # vel.linear.x = v
            vx, vy, w = self.controller.get_drone_cmdVel(self.drone_pose, self.waypoint_x, self.waypoint_y)
            vel.angular.z = w
            vel.linear.x = vx
            vel.linear.y = vy

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
