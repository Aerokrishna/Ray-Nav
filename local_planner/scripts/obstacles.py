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

        self.get_logger().info("OBSTACLE DETECTION ENABLED")

    def scan_callback(self, laser_msg : LaserScan):
        range_data = laser_msg.ranges
        # self.get_logger().info(f"angle_incr :{laser_msg.angle_increment}, max_range:{laser_msg.range_max}, length:{len(range_data)}")

        max_range = 5

        self.obstacle_x = []
        self.obstacle_y = []

        # go through all the points in the point cloud array        
        for i in range(1,len(range_data)):
            j = i + 1
            if j > len(range_data)-1:
                j = 0

            range_val = range_data[i]
            theta = laser_msg.angle_min + (i * laser_msg.angle_increment)
            
            if (range_data[i-1] > max_range and range_data[i] < max_range):
                
                local_x = range_val * math.cos(theta)
                local_y = range_val * math.sin(theta)

                global_x,global_y = self.transform_point(local_x,local_y)
                global_theta = math.atan2(global_y - self.robot_y, global_x - self.robot_x)

                inf_x = self.inf_radius * math.sin(global_theta)
                inf_y = self.inf_radius * math.cos(global_theta)

                global_x += inf_x
                global_y -= inf_y

                self.obstacle_x.append(global_x)
                self.obstacle_y.append(global_y)
            
            elif (range_data[i] < max_range and range_data[j] > max_range):
                
                local_x = range_val * math.cos(theta)
                local_y = range_val * math.sin(theta)

                global_x,global_y = self.transform_point(local_x,local_y)
                global_theta = math.atan2(global_y - self.robot_y, global_x - self.robot_x)

                inf_x = self.inf_radius * math.sin(global_theta)
                inf_y = self.inf_radius * math.cos(global_theta)

                global_x -= inf_x
                global_y += inf_y

                self.obstacle_x.append(global_x)
                self.obstacle_y.append(global_y)

        # self.get_logger().info(f"(X,Y) = ({self.obstacle_x,self.obstacle_y})")

        obs_msg = Obstacle()
        obs_msg.obstacles_x = self.obstacle_x
        obs_msg.obstacles_y = self.obstacle_y

        self.obstacle_publisher_.publish(obs_msg)

    def odom_callback(self, odom: Odometry):
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y

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

            marker_msg.id = i+1
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
            return 0.0,0.0

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