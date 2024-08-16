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
from local_planner_interfaces.msg import Obstacle
from tf_transformations import euler_from_quaternion
import numpy as np

class IdealWaypoint(Node):
    def __init__(self):
        super().__init__('lser_scan')

        # publishers and subscribers initialization
        self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.obstacle_subscriber = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        # self.marker_publisher_ = self.create_subscription(Obstacle,'obstacles',self.obstacle_callback,10)
        
        # self.marker_timer = self.create_timer(0.1, self.marker_timer)
        self.tf_timer = self.create_timer(0.1, self.transform_point)

        # transformation 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.range_data = []
        self.goal = (5,5)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.get_logger().info("WAYPOINT COMPUTER ENABLED")

    def scan_callback(self, laser_msg : LaserScan):
        range_data = laser_msg.ranges
        max_range = 3
        waypoint_thres = 1

        goal_or = self.compute_vector((0,0),(self.goal[0],self.goal[1]))

        scan_or = abs(goal_or - self.robot_yaw)
        scan_or = np.rad2deg(scan_or)
        scan_index = round(scan_or)
        safe = 0

        for i in range(scan_index - 7,scan_index + 7):
            if i > len(range_data)-1:
                i -= len(range_data)

            if range_data[i] < max_range:
                safe+=1

        if safe == 0:
            waypoint = (waypoint_thres * np.cos(goal_or), waypoint_thres * np.sin(goal_or))
            self.get_logger().info(f"waypoint : {waypoint}")

        # self.get_logger().info(f"scan index {scan_index}")

    def compute_vector(self, vect1, vect2):
        return np.arctan2((vect2[1] - vect1[1]),(vect2[0] - vect1[0]))
    
    def obstacle_callback(self,obs_msg: Obstacle):
        self.obstacle_x = obs_msg.obstacles_x
        self.obstacle_y = obs_msg.obstacles_y

    def odom_callback(self, odom: Odometry):
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y

        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w

        self.robot_yaw = euler_from_quaternion([x,y,z,w])[2]

    # function to publish visualization points on Rviz
    def marker_timer(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = "/odom" # wrt which frame we are taking the coordinates
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.type = Marker.SPHERE
    
        marker_msg.scale.x = 0.07
        marker_msg.scale.y = 0.07
        marker_msg.scale.z = 0.07

        marker_msg.color.r = 0.0
        marker_msg.color.g = 255.0
        marker_msg.color.b = 125.0
        marker_msg.color.a = 1.0

        for i in range(len(self.obstacle_x)):
          
            marker_msg.pose.position.x = self.obstacle_x[i]
            marker_msg.pose.position.y = self.obstacle_y[i]
            marker_msg.pose.orientation.w = 1.0

            marker_msg.id = i
            self.marker_publisher_.publish(marker_msg)      
        
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

    def get_distance(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
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