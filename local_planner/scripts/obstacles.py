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
from tf_transformations import euler_from_quaternion
from local_planner_interfaces.msg import Obstacle
import numpy as np

class DetectObstacles(Node):
    def __init__(self):
        super().__init__('obstacles')

        # publishers and subscribers initialization
        self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.marker_publisher_ = self.create_publisher(Marker,'visualization_marker',10)
        self.obstacle_publisher_ = self.create_publisher(Obstacle,'obstacles',10)
        
        # self.marker_timer = self.create_timer(0.1, self.marker_timer)
        self.tf_timer = self.create_timer(0.1, self.transform_point)

        # transformation 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.obstacle_x = []
        self.obstacle_y = []
        self.inf_radius = 0.3

        self.robot_yaw = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.get_logger().info("OBSTACLE DETECTION ENABLED")

    '''
    A function to process scan data and publish obstacle edges
    '''

    def scan_callback(self, laser_msg : LaserScan):
        self.range_data = laser_msg.ranges
        self.angle_increament = laser_msg.angle_increment
        self.angle_min = laser_msg.angle_min

        max_range = 5

        self.obstacle_x = []
        self.obstacle_y = []

        for i in range(1,len(self.range_data)):
            j = i + 1
            if j > len(self.range_data)-1:
                j = 0

            range_val = self.range_data[i]
            theta = self.angle_min + (i * self.angle_increament)
            
            if (self.range_data[i-1] > max_range and self.range_data[i] < max_range):

                result, obs_range, obs_theta = self.check_inflation(range_val, theta, -1)

                if result == True:
                    # means this point is safe
                    obs_x, obs_y = self.scan_to_pose(obs_range, obs_theta)[:2]
                    self.obstacle_x.append(obs_x)
                    self.obstacle_y.append(obs_y)
            
            elif (self.range_data[i] < max_range and self.range_data[j] > max_range):

                result, obs_range, obs_theta = self.check_inflation(range_val, theta, 1)

                if result == True:
                    # means this point is safe
                    obs_x, obs_y = self.scan_to_pose(obs_range, obs_theta)[:2]
                    self.obstacle_x.append(obs_x)
                    self.obstacle_y.append(obs_y)

        # self.get_logger().info(f"(X,Y) = ({self.obstacle_x,self.obstacle_y})")

        obs_msg = Obstacle()
        obs_msg.obstacles_x = self.obstacle_x
        obs_msg.obstacles_y = self.obstacle_y

        self.obstacle_publisher_.publish(obs_msg)

    '''
    A function to check if the obstacle edge is big enough to pass through
    '''
    def check_inflation(self, point_range, theta, side):
        
        scan_or = np.rad2deg(theta)
        scan_index = round(scan_or)
        index = round(np.rad2deg(0.6/point_range))
        # a =[]
        safe = 0
        wp_range = point_range
        wp_theta = 0

        for i in range(scan_index ,scan_index + (side * index), side):
            if i > len(self.range_data)-1:
                i -= len(self.range_data)

            if i == int((scan_index + (side * (index/2)))):
                wp_theta = self.angle_min + (i * self.angle_increament)

            if self.range_data[i] < point_range:
                safe+=1
            # a.append(self.range_data[i])
        if safe <= 1:
            return [True, wp_range, wp_theta]

        return [False, wp_range, wp_theta]

    
    '''
    Odometry Callback
    '''
    def odom_callback(self, odom: Odometry):
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w

        self.robot_yaw = euler_from_quaternion([x,y,z,w])[2]

    '''
    Visualization timer
    '''
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

            marker_msg.id = i+1
            self.marker_publisher_.publish(marker_msg)

    '''
    A helper fuction to transform point from base link to odom frame
    '''
    def transform_point(self, local_x = 0.0, local_y = 0.0):
        local_point = PointStamped()
        local_point.header.frame_id = "base_link"
        
        local_point.point.x = local_x  
        local_point.point.y = local_y  
        local_point.point.z = 0.0  
        
        now = self.get_clock().now()

        try:
            self.tf_buffer.lookup_transform("odom", "base_link",Time(nanoseconds=0),Duration(seconds=1.0))
            
            global_point = self.tf_buffer.transform(local_point, "odom")

            return global_point.point.x,global_point.point.y

        except:
            self.get_logger().error("error!")
            return 0.0,0.0

    '''
    A function to convert scan ranges to global points
    '''
    def scan_to_pose(self, range_val, theta):
        local_x = range_val * math.cos(theta)
        local_y = range_val * math.sin(theta)

        global_x,global_y = self.transform_point(local_x,local_y)
        global_theta = math.atan2(global_y - self.robot_y, global_x - self.robot_x)

        return global_x, global_y, global_theta
    
    '''
    A function to compute vector between two points
    '''
    def compute_vector(self, vect1, vect2):
        return np.arctan2((vect2[1] - vect1[1]),(vect2[0] - vect1[0]))
    
    '''
    A helper function to get distance between two points
    '''
    def get_distance(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
def main(args=None):
    rclpy.init(args=args)
    node = DetectObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()