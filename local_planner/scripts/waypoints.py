#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import PointStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import Odometry
import math
from local_planner_interfaces.msg import Obstacle, Waypoint
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
import numpy as np
import time

class IdealWaypoint(Node):
    def __init__(self):
        super().__init__('waypoints')

        # publishers and subscribers initialization
        # self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        # self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        self.drone_lidar_subscriber = self.create_subscription(LaserScan,'/drone/scan',self.scan_callback,10)
        self.drone_odom_subscriber = self.create_subscription(Pose,'/drone/gt_pose',self.drone_odom_callback,10)

        self.obs_subscriber = self.create_subscription(Obstacle,'/obstacles',self.obstacle_callback,10)
        self.marker_publisher_ = self.create_publisher(Marker,'/visualization_marker',10)
        self.waypoint_publisher_ = self.create_publisher(Waypoint,'/waypoints',10)
        
        self.marker_timer_ = self.create_timer(0.3, self.marker_timer)

        self.waypoint_timer_ = self.create_timer(0.1, self.waypoint_timer)
        self.tf_timer = self.create_timer(0.1, self.transform_point)

        # transformation 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.range_data = []
        self.goal = (5.0,5.0)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.range_data = []
        self.max_range = 5
        
        self.get_logger().info("WAYPOINT COMPUTER ENABLED")

        self.obstacle_x = []
        self.obstacle_y = []
        self.odom_points = []

        self.cnt = 0

        self.wpx = 0.0
        self.wpy = 0.0
        self.ox = 0.0
        self.oy = 0.0

    '''
    A function to get the obstacle edges
    '''
    def obstacle_callback(self,obs_msg: Obstacle):
        self.obstacle_x = obs_msg.obstacles_x
        self.obstacle_y = obs_msg.obstacles_y

        # print(self.obstacle_x,self.obstacle_y)

    '''
    A function to publish the waypoint
    '''
    def waypoint_timer(self):
        self.cnt += 1
        if self.cnt > 10:
            waypoint = Waypoint()

            self.wpx, self.wpy = self.get_waypoint()

            waypoint.waypoint_x = self.wpx 
            waypoint.waypoint_y = self.wpy

            # self.get_logger().info(f"WAYPOINTS : {waypoint.waypoint_x, waypoint.waypoint_y}")
            self.waypoint_publisher_.publish(waypoint)
            self.cnt = 10
    '''
    Lidar scan call back
    '''
    def scan_callback(self, laser_msg : LaserScan):

        self.range_data = laser_msg.ranges
    
    '''
    To compute the waypoint with the lowest cost
    '''
    def get_waypoint(self):

        if self.get_goal_ray():
            # self.get_logger().info("GOAL MODE ACTIVATED")
            goal_or = self.compute_vector((self.robot_x,self.robot_y),(self.goal[0],self.goal[1]))

            gx = self.robot_x + np.cos(goal_or)
            gy = self.robot_y + np.sin(goal_or)

            if self.get_distance(self.robot_x,self.robot_y,self.goal[0],self.goal[1]) < 1.0:
                gx,gy = self.goal

            return gx,gy
        
        obstacle_cost = {}
        safe_wp_x = []
        safe_wp_y = []

        for i in range(len(self.obstacle_x)):
            
            obs_x = self.obstacle_x[i]
            obs_y = self.obstacle_y[i]

            if self.get_distance(self.robot_x,self.robot_y,self.goal[0],self.goal[1]) > 1.0:
                obs_or = self.compute_vector((self.robot_x,self.robot_y),(obs_x,obs_y))

                obs_x = self.robot_x + np.cos(obs_or)
                obs_y = self.robot_y + np.sin(obs_or)

            safe_wp_x.append(obs_x)
            safe_wp_y.append(obs_y)

            obstacle_cost[self.get_cost(obs_x,obs_y)] = i

        sorted_cost = list(obstacle_cost.keys())
        sorted_cost.sort()
        
        sorted_obstacle_cost = {i: obstacle_cost[i] for i in sorted_cost}
        
        sorted_keys = list(sorted_obstacle_cost.values())

        obs_x = safe_wp_x[sorted_keys[0]]
        obs_y = safe_wp_y[sorted_keys[0]]

        self.ox = self.obstacle_x[sorted_keys[0]]
        self.oy = self.obstacle_y[sorted_keys[0]]

        return obs_x,obs_y

    '''
    To check if the path to goal is obstacle free or not
    '''
    def get_goal_ray(self):
        # self.get_logger().info("GET RAY")
        r = 1.5
        k = self.get_distance(self.robot_x,self.robot_y,self.goal[0],self.goal[1])
        wp_or = self.compute_vector((self.robot_x, self.robot_y),(self.goal[0], self.goal[1]))

        if k < 1:
            r = k
        index = 30

        scan_or = wp_or - self.robot_yaw
        if scan_or < 0:
            scan_or+=(2*np.pi)

        scan_or = np.rad2deg(scan_or)
        scan_index = round(scan_or)

        # a =[]
        safe = 0
        for i in range(scan_index - index,scan_index + index):
            if i > len(self.range_data)-1:
                i -= len(self.range_data)

            if self.range_data[i] < r:
                safe+=1
            # a.append(self.range_data[i])
        
        if safe == 0: 
            return True
        else: 
            return False
        
    '''
    To compute the cost of a potential waypoint
    '''
    def get_cost(self,obs_x,obs_y):

        dr = self.get_distance(self.robot_x, self.robot_y, obs_x, obs_y)
        dg = self.get_distance(obs_x, obs_y, self.goal[0], self.goal[1])

        obs_or = self.compute_vector((self.robot_x, self.robot_y),(obs_x, obs_y))
        control_effort = abs(1/obs_or)
        
        cost = (dg)
        return cost
    
    def odom_callback(self, odom: Odometry):
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y

        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w

        self.robot_yaw = euler_from_quaternion([x,y,z,w])[2]
    
    def drone_odom_callback(self, odom: Pose):
        self.robot_x = odom.position.x
        self.robot_y = odom.position.y

        x = odom.orientation.x
        y = odom.orientation.y
        z = odom.orientation.z
        w = odom.orientation.w

        self.robot_yaw = euler_from_quaternion([x,y,z,w])[2]

    # function to publish visualization points on Rviz
    def marker_timer(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = "/odom" # wrt which frame we are taking the coordinates
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.type = Marker.SPHERE
    
        marker_msg.scale.x = 0.13
        marker_msg.scale.y = 0.13
        marker_msg.scale.z = 0.13

        marker_msg.color.r = 0.0
        marker_msg.color.g = 255.0
        marker_msg.color.b = 125.0
        marker_msg.color.a = 1.0

        for i in range(len(self.obstacle_x)):
          
            marker_msg.pose.position.x = self.obstacle_x[i]
            marker_msg.pose.position.y = self.obstacle_y[i]
            marker_msg.pose.position.z = 2.1

            marker_msg.pose.orientation.w = 1.0

            marker_msg.id = i+2
            self.marker_publisher_.publish(marker_msg)
        
        marker_msg.type = Marker.SPHERE
    
        marker_msg.scale.x = 0.13
        marker_msg.scale.y = 0.13
        marker_msg.scale.z = 0.13

        marker_msg.color.r = 0.0
        marker_msg.color.g = 255.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0

        marker_msg.pose.position.x = self.wpx
        marker_msg.pose.position.y = self.wpy
        marker_msg.pose.position.z = 2.10

        marker_msg.pose.orientation.w = 1.0

        marker_msg.id = 1
        self.marker_publisher_.publish(marker_msg)

        # marker_msg.scale.x = 0.09
        # marker_msg.scale.y = 0.09
        # marker_msg.scale.z = 0.09

        # marker_msg.color.r = 0.0
        # marker_msg.color.g = 125.0
        # marker_msg.color.b = 255.0
        # marker_msg.color.a = 1.0

        # marker_msg.pose.position.x = self.wpx
        # marker_msg.pose.position.y = self.wpy
        # marker_msg.pose.position.z = 2.10

        # marker_msg.pose.orientation.w = 1.0

        # marker_msg.id = 0
        # self.marker_publisher_.publish(marker_msg)

        # marker_msg.type = Marker.SPHERE
    
        # marker_msg.scale.x = 0.06
        # marker_msg.scale.y = 0.06
        # marker_msg.scale.z = 0.06

        # marker_msg.color.r = 255.0
        # marker_msg.color.g = 125.0
        # marker_msg.color.b = 0.0
        # marker_msg.color.a = 1.0

        # self.odom_points.append((self.robot_x,self.robot_y))

        # for i in range(len(self.odom_points)):
        #     marker_msg.pose.position.x = self.odom_points[i][0]
        #     marker_msg.pose.position.y = self.odom_points[i][1]
        #     marker_msg.pose.orientation.w = 1.0
        
        #     marker_msg.id = i + 1

        #     self.marker_publisher_.publish(marker_msg)
              
        
    # tranformation function
    def transform_point(self, local_x = 0.0, local_y = 0.0):
        local_point = PointStamped()
        local_point.header.frame_id = "base_link"
        
        local_point.point.x = local_x  # Replace with your actual X coordinate
        local_point.point.y = local_y  # Replace with your actual Y coordinate
        local_point.point.z = 0.0  # Replace with your actual Z coordinate
        
        now = self.get_clock().now()

        try:
            # Wait for the transform to become available for up to 1 second
            self.tf_buffer.lookup_transform("odom", "base_link",Time(nanoseconds=0),Duration(seconds=1.0))
            
            global_point = self.tf_buffer.transform(local_point, "odom")

            return global_point.point.x,global_point.point.y

        except:
            self.get_logger().error("error!")
            pass
    
    def compute_vector(self, vect1, vect2):
        return np.arctan2((vect2[1] - vect1[1]),(vect2[0] - vect1[0]))
    
    def get_distance(self,x1,y1,x2,y2):
        return np.sqrt((x2-x1)**2 + (y2-y1)**2)
    
def main(args=None):
    rclpy.init(args=args)
    node = IdealWaypoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()