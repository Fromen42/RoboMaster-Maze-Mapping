from sre_constants import RANGE
import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Quaternion

import sys
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.stages = {
            # 1: move towards the wall
            1: {
                "linear": 0.13,
                "angular": -0.3
            },
            # 2: turn towards the wall
            2: {
                "linear": 0.13,
                "angular": 0.3

            }, 
            # 3: turn 90 degrees away from wall FAST
            3: {
                "linear": 0.2,
                "angular": 0.0
            },
            # 4: turn -90 degrees away from wall FAST
            4: {
                "linear": 0.0,
                "angular": 0.5
            },

            # 5: turn 180 degrees away from wall SLOW (for accuracy)
            5: {
                "linear": 0.0,
                "angular": 0.06
            },
            # 6: move away from wall
            6: {
                "linear": 1.0,
                "angular": 0.0
            },
            # 7: stop 2m away from wall
            7: {
                "linear": 0.0,
                "angular": 0.0
            }
        }
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        self.center = -1 # base sensor number
        self.center_right = -1 # base sensor number
        self.center_left = -1 # base sensor number

        self.rear_right = -1 # base sensor number
        self.rear_left = -1# base sensor number

        self.distance_to_wall = None # first check to see how close it is to the wall

        self.start_at_wall = 0 # pose at wall

        self.turn_clicks = 180 # number of ticks it should turn FAST (not good coding practice)

        self.turn_left = 0 # number of consecutive left turns
        self.turn_right = 0 # number of consecutive right turns

        self.distance_rear_to_wall = None # check to see how close rear is to the wall
        self.stage = 1 # starting stage is always 1

        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.center_subscriber = self.create_subscription(Range, 'range_3', self.range_center_callback, 10)
        self.center_right_subscriber = self.create_subscription(Range, 'range_1', self.range_center_right_callback, 10)
        self.center_left_subscriber = self.create_subscription(Range, 'range_3', self.range_center_left_callback, 10)

        self.back_right_subscriber = self.create_subscription(Range, 'range_0', self.range_rear_right_callback, 10)
        self.back_left_subscriber = self.create_subscription(Range, 'range_2', self.range_rear_left_callback, 10)

        self.orient_subscriber = self.create_subscription(Quaternion, 'imu', self.quaternion_orient_callback, 10)

        

        
    def start(self):
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def range_center_callback(self, msg):
        self.center = msg.range # center range

    def range_center_right_callback(self, msg):
        self.center_right = msg.range

    def range_center_left_callback(self, msg):
        self.center_left = msg.range

    def range_rear_right_callback(self, msg):
        self.rear_right = msg.range
    
    def range_rear_left_callback(self, msg):
        self.rear_left = msg.range

    def quaternion_orient_callback(self, msg):
        self.orientation = msg.quaternion

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        self.pose2d = pose2d
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2

    def determine_linear_velocity(self):
        return self.stages[self.stage]['linear']

    def determine_angular_vel(self):
        return self.stages[self.stage]['angular']

    def stage5_exit(self):
        euclidean_distance = math.sqrt(((self.start_at_wall[0]-self.pose2d[0])**2+(self.start_at_wall[1]-self.pose2d[1])**2))
        if abs(self.distance_rear_to_wall + euclidean_distance - 2) < 0.05:
            return True
        return False

    
    def turn_clicker(self):
        self.turn_clicks -= 1
        return self.turn_clicks

    def click_reset(self):
        self.turn_clicks = 130
        return self.turn_clicks

    def count_left(self):
        self.turn_left += 1;
        return self.turn_left

    def count_right(self):
        self.turn_right += 1;
        return self.turn_right

    def count_left_reset(self):
        self.turn_left = 0;
        return self.turn_left

    def count_right_reset(self):
        self.turn_right = 0;
        return self.turn_right


    def determine_stage(self):
        # 1 turning Right to the wall
        if self.stage == 1 and abs(self.center) < 0.35 : # Wall in front jump to stage 4
            
            self.click_reset()
            self.stage += 3
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)
            
            
        elif self.stage == 1 and self.center_right < 0.22 : # Too close to the wall on Right jump to stage 2
            self.click_reset()
            self.stage += 1
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)

        elif self.stage == 1 and abs(self.center_right - self.rear_right) < 0.005 : # parallel to the wall on Right jump to stage 3
            self.click_reset()
            self.stage += 2
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)

        # 2 turning Left away from the wall

        if self.stage == 2 and self.center_right > 0.25: # Too far away from the wall jump to stage 1
            self.distance_to_wall = self.center
            self.click_reset()
            self.stage -= 1
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)

        elif self.stage == 2 and self.center < 0.35 : # Wall in front jump to stage 4
            self.click_reset()
            self.stage += 2 
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)
        
        # 3 Go straight

        if self.stage == 3 and self.center < 0.35 : # Wall in front jump to stage 4
            self.count_left() 
            self.count_right_reset()
            self.click_reset()
            self.stage += 1
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)

        elif self.stage == 3 and self.center_right < 0.18: # too close jump stage 2 
            self.click_reset()
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)
            self.stage -= 1
        
        elif self.stage == 3 and self.center_right > 0.40: # too far jump stage 1
            self.click_reset()
            self.stage -= 2
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)

        
        # 4 Turn 90 degrees 

        if self.stage == 4 and self.turn_clicker() <= 0 and abs(self.center_right - self.rear_right) < 0.005 :
            self.count_right()
            self.count_left_reset()
            self.click_reset()
            self.stage -= 1
            self.get_logger().info(f"GOING TO STAGE: {self.stage}",throttle_duration_sec=0.5)
        

         

    def update_callback(self):
        cmd_vel = Twist() 

        self.determine_stage()
        cmd_vel.linear.x  = self.determine_linear_velocity()
        cmd_vel.angular.z = self.determine_angular_vel()
        
        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
