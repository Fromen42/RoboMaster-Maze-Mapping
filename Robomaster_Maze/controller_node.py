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
                "angular": 0.5
            },
            # 6: move away from wall
            6: {
                "linear": 0.0,
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
        self.quaternion_pose = None

        self.pose2d = (0,0,0)

        self.turn_start_pose = 0
        self.turn_pose = 0

        self.sensors = [Sensor('front_right', [0.0,0.2,math.pi], 5, -1), Sensor('front',[0.0,0.2,(3*math.pi/2)], 5, -1), Sensor('back_right',[-0.1,-0.2,0], 5, -1), Sensor("back_left",[0.0,-0.2,0], 5, -1),]
        
        self.map = Localization(self.sensors,0.1)

        self.index = 0

        

        
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
        self.quaternion_pose = msg.quaternion
        pose2d = self.pose3d_to_2d(self.quaternion_pose)
        self.quaternion_pose2d = pose2d

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        self.turn_pose = pose2d[2]
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

    def pose_reset(self):
        self.turn_start_pose = self.turn_pose
        return self.turn_start_pose

    def turn_left_pose(self):
        return abs(self.turn_start_pose - self.turn_pose)

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
        ''''''
        self.map.update_current_pose([self.pose2d[0], self.pose2d[1], self.pose2d[2]])

        # self.index = self.index +1
        self.map.print_map(self.index)

        # 1 turning Right to the wall
        if self.stage == 1 and abs(self.center) < 0.35 : # Wall in front jump to stage 4
            self.click_reset()
            self.pose_reset()
            self.stage += 3
            
        elif self.stage == 1 and self.center_right < 0.23 : # Too close to the wall on Right jump to stage 2
            self.click_reset()
            self.stage += 1

        elif self.stage == 1 and abs(self.center_right - self.rear_right) < 0.005 : # parallel to the wall on Right jump to stage 3
            self.click_reset()
            self.stage += 2

        elif self.stage == 1 and self.center_right - self.rear_right < 0 : # parallel to the wall on Right jump to stage 3
            self.click_reset()
            self.stage += 1
        
        elif self.stage == 1:
            self.map.insert_sensor_point("front_right", self.center_right)
            self.map.insert_sensor_point("back_left", self.rear_left)

        # 2 turning Left away from the wall

        if self.stage == 2 and self.center_right > 0.25: # Too far away from the wall jump to stage 1
            self.distance_to_wall = self.center
            self.click_reset()
            self.stage -= 1


        elif self.stage == 2 and self.center < 0.35 : # Wall in front jump to stage 4
            self.click_reset()
            self.pose_reset()
            self.stage += 2 

        elif self.stage == 2 and abs(self.center_right - self.rear_right) < 0.005 : # parallel to the wall on Right jump to stage 3
            self.click_reset()
            self.stage += 1


        elif self.stage == 2:
            self.map.insert_sensor_point("front_right", self.center_right)
            self.map.insert_sensor_point("back_left", self.rear_left)
        
        # 3 Go straight

        if self.stage == 3 and self.center < 0.35 and self.center_right - self.rear_right < 0 : # Wall in front jump to stage 4
            self.pose_reset()
            self.stage += 1

        elif self.stage == 3 and self.center < 0.35 and self.center_right - self.rear_right >= 0 : # Wall in front jump to stage 4
            self.pose_reset()
            self.stage += 2

        elif self.stage == 3 and self.center_right < 0.18: # too close jump stage 2 
            self.click_reset()
            self.stage -= 1
        
        elif self.stage == 3 and self.center_right > 0.40: # too far jump stage 1
            self.click_reset()
            self.stage -= 2
        
        elif self.stage == 3:
            self.map.insert_sensor_point("front_right", self.center_right)
            self.map.insert_sensor_point("back_left", self.rear_left)
        
        # 4 Turn 90 degrees or more

        if self.stage == 4 and self.turn_left_pose() >= (3*math.pi/8) and abs(self.center_right - self.rear_right) < 0.01 :
            self.count_right()
            self.count_left_reset()
            self.click_reset()
            self.stage -=3


        elif self.stage == 4:
            self.map.insert_sensor_point("front_right", self.center_right)
            self.map.insert_sensor_point("back_left", self.rear_left)

        # 5 Turn 90 degrees or less

        if self.stage == 5 and abs(self.center_right - self.rear_right) < 0.01 :
            self.count_right()
            self.count_left_reset()
            self.click_reset()
            self.stage -=4


        elif self.stage == 5:
            self.map.insert_sensor_point("front_right", self.center_right)
            self.map.insert_sensor_point("back_left", self.rear_left)
        

    def update_callback(self):
        ''''''
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

class Sensor:
    def __init__(self, name, sensor_pose, max_distance, default_value=-1):
        self.name = name
        self.pose = [-sensor_pose[0], -sensor_pose[1],-sensor_pose[2]]
        self.max_distance = max_distance # the maximum distance that the sensor can sense
        self.default_value = default_value # the value that is returned when it finds nothing
        
class Localization:
    def __init__(self, sensors, precision=0.05):
        '''
        Layout of the squares to form the complete localization matrix
                        1  |  0
                        -------
                        3  |  2
        
        '''
        
        self.sensors = {}
        for s in sensors:
            self.sensors[s.name] = s
            
        self.precision = precision
        self.pose = [0, 0, 0] # x, y, theta
        self.pose_coords = self.get_matrix_coords(self.pose) # [0, (0, 0)]: yaw, (x, y)
        # these are our localization matrixies
        # each represents the square of the euclidean space, when split along x=0, y=0
        self.top_left = []
        self.top_right = []
        self.bottom_right = []
        self.bottom_left = []
        
        self.map = [self.top_right, self.top_left, self.bottom_right, self.bottom_left] # selected like this, to amke selecting square easier
        self.map_print = None
        self.matrix_map = []
    
        self.max_dim = 0
        
        self.vertical_square_trans = {
            0: 2,
            1: 3,
            3: 1,
            2: 0
        }
    # note that the current+pose includes 3 coordinates: x, y and theta (the yaw)
    def get_absolute_position(self, current_pose, distance):
        '''
        Given a the pose of a robot, and the distance that a sensor detects,
        computes the absolute position of the obstruction
        '''
        x = distance * math.sin(current_pose[2])
        y = distance * math.cos(current_pose[2])
        
        return [x+current_pose[0], y+current_pose[1]]
    
    def update_current_pose(self, pose):
        self.pose = pose
        self.pose[2] = -self.pose[2]
        self.pose_coords = self.get_matrix_coords(self.pose)
        try:
            self.verify_matrix_sizes(self.pose_coords[0], self.pose_coords[1])
            self.insert_value_map(self.pose_coords[0], self.pose_coords[1], 3, True)
        except:
            print("Pose position map update failed: Pose position not discovered yet. Consider running 'verify_matrix_sizes' before the previous code block")
        
    def round_coordinate(self, coor):
        '''rounds a single coordinate to the nearest position'''
        return round(coor / self.precision)*self.precision
    
    def round_coordinates(self, coordinate):
        '''rounds the given values to the precision given. Handles any dimensional array'''
        final_coordinates = []
        for coor in coordinate:
            final_coordinates.append(self.round_coordinate(coor))
        return final_coordinates, 
    
    def get_square_number(self, abs_position):
        '''
        selects the square in which the point is in:
                    1  |  0
                    -------
                    3  |  2
        '''
        square_number = 0
        square_number += 1 if abs_position[0] < 0 else 0
        square_number += 2 if abs_position[1] < 0 else 0
        return square_number
    
    def get_matrix_coords(self, abs_position):
        '''gets the matrix coordinates of the position in the matrix where the data point is located'''
        # getting the position of the matrix
        square_number = self.get_square_number(abs_position)
        
        # determining the positive value 
        matrix_position = [abs_position[0], abs_position[1]]
        
        if matrix_position[0] < 0:
            matrix_position[0] *= -1
        if matrix_position[1] < 0:
            matrix_position[1] *= -1

        matrix_position = [int((self.round_coordinate(coor / self.precision))) for coor in matrix_position]

        for i in range(2):
            if matrix_position[i] < 0:
                matrix_position[i] = 0
        return [square_number, matrix_position]
    
    def expand_rows(self, square, row_number):
        '''If the rows in the localization matrix don't suffice,
        we expand the matrix to accomodate more location points'''
        to_add = row_number - len(self.map[square]) + 1
        for i in range(to_add):
            row = []
            for j in range(self.max_dim):
                row.append(0)
            self.map[square].append(row)
            
    def expand_column(self, square, row_num):
        '''If the columns in one of the rows in the localization matrix don't suffice,
        we expand the matrix to accomodate more location points in the y direction'''
        to_add = self.max_dim - len(self.map[square][row_num]) + 1
            
        for i in range(to_add):
            self.map[square][row_num].append(0)

    def verify_matrix_sizes(self, square, new_pos):
        '''resizes all squares based off o fhte maximum value in any dimension, in any square'''
        # to add the appropriate unmber of columns fo rows
        self.max_dim = 0
        if new_pos[0] > self.max_dim:
            self.max_dim = new_pos[0]
        if new_pos[1] > self.max_dim:
            self.max_dim = new_pos[1]
        
        for square in range(4):
            if self.max_dim > len(self.map[square])-1:
                self.expand_rows(square, self.max_dim)
            for i in range(len(self.map[square])):
                if self.max_dim > len(self.map[square][i])-1:
                    self.expand_column(square, i)

    
    def fill_lines(self, lines):
        for line in lines:
            points_of_line = Helpers.get_line_bresenham(line[0], line[1])
            for point in points_of_line:
                self.insert_value_map(line[2], point, 2)
    
    def get_slope(self, p1, p2):
        if (p2[0] - p1[0] != 0):
            return (p2[1] - p1[1]) / (p2[0] - p1[0])
        else:
            return 0
    
    def get_x_intercept(self, p1, p2):
        m = self.get_slope(p1, p2)
        c = p1[1]-m*p1[0]
        return (-c/m, 0)
    
    def get_y_intercept(self, p1, p2):
         m = self.get_slope(p1, p2)
         c = -m*p1[0] + p1[1]
         return (0, c)

    
    def fill_in_line_of_sight(self, position_square, matrix_position, absolute_position, sensor_pose):
        '''if one of the sensors detects something, it means that the path to that obstruction is clear.
        This function fills that line up with empty values. Squares:
        
                        1  |  0
                        -------
                        3  |  2
        '''

        lines_to_fill = []
        
        s_pose_coords = sensor_pose[0]
        s_pose =  sensor_pose[1] # pose coords of the sensor

        if s_pose_coords[0] == position_square:
            lines_to_fill.append([s_pose_coords[1], matrix_position, s_pose_coords[0]]) 
            return self.fill_lines(lines_to_fill)
        
        x_intercept = None
        y_intercept = None
        
        
        if abs(s_pose_coords[0] - position_square) == 2:

            # one position is up, one is down. Must find x_intercept
            if s_pose[0] == absolute_position[0]:
                # vertical line. can simply draw the vertical line
                lines_to_fill.append([s_pose_coords[1][:2], (s_pose_coords[1][0], 0), s_pose_coords[0]]) # 
                lines_to_fill.append([(s_pose_coords[1][0], 0), matrix_position, position_square])

            else:
                # must calculate slope, find the intercept
                x_intercept = self.get_x_intercept(s_pose, absolute_position)
                intercept_coords = self.get_matrix_coords(x_intercept)
                
                lines_to_fill.append([s_pose_coords[1], intercept_coords[1], s_pose_coords[0]]) # 
                lines_to_fill.append([intercept_coords[1], matrix_position, position_square])
                
        elif abs(s_pose_coords[0] - position_square) == 1 and sum([s_pose_coords[0], position_square]) != 3:

            # one position is right, one is left: Line goes horizontal (but not diagonal!). There is a slope
            y_intercept = self.get_y_intercept(s_pose[:2], absolute_position)
            intercept_coords = self.get_matrix_coords(y_intercept)
            
            lines_to_fill.append([s_pose_coords[1], intercept_coords[1], s_pose_coords[0]]) # 
            lines_to_fill.append([intercept_coords[1], matrix_position, position_square])
             
        elif abs(s_pose_coords[0] + position_square) == 3:

            # is diagonal. We will need both intersection points
            x_intercept = self.get_x_intercept(s_pose, absolute_position)
            y_intercept = self.get_y_intercept(s_pose, absolute_position)
            
            x_intercept_coords = self.get_matrix_coords(x_intercept)
            y_intercept_coords = self.get_matrix_coords(y_intercept)
            
            if x_intercept_coords[0] == y_intercept_coords[0] and x_intercept_coords[1] == y_intercept_coords[1] :
                
                # intersect point is at (0,0). Thus, we only need 2 lines
                lines_to_fill.append([s_pose_coords[1], [0,0], s_pose_coords[0]]) # 
                lines_to_fill.append([[0,0], matrix_position, position_square])
                
            else:
                # There will be 3 lines to be drawn.
                # need to determine if x or y intercept comes first the pose (or position)
                x_intercept_real = self.apply_square_multipliers(x_intercept_coords)
                y_intercept_real = self.apply_square_multipliers(y_intercept_coords)

                slope_points = self.get_slope(x_intercept_coords[1], y_intercept_coords[1])
                x_smaller = x_intercept_real[0] < y_intercept_real[0]
                order = None

                
                # checking to see what square is in
                if slope_points > 0:
                    # in either 3 or 0
                    if s_pose[0] > 0:
                        # in 0
                        if x_smaller:
                            # x intercept
                            x_intercept_coords[0] = 2
                            order =  [x_intercept_coords, y_intercept_coords]
                        else:
                            # y intercept 
                            y_intercept_coords[0] = 1
                            order =  [y_intercept_coords, x_intercept_coords]
                        
                    else:
                        # in 3
                        if x_smaller:
                            # x intersept
                            x_intercept_coords[0] = 1
                            order =  [x_intercept_coords, y_intercept_coords]
                        else:
                            
                            # y intercept
                            y_intercept_coords[0] = 2
                            order =  [y_intercept_coords, x_intercept_coords]


                else:
                    # in either 1 or 2
                    if s_pose[0] > 0:
                        # in 2
                        if x_smaller:
                            # y intercept
                            y_intercept_coords[0] = 3
                            order =  [y_intercept_coords, x_intercept_coords]
                            
                        else:
                            # x intercept
                            x_intercept_coords[0] = 0
                            order =  [x_intercept_coords, y_intercept_coords]
                    else:
                        # in 1
                        if x_smaller:
                            # x intercept
                            x_intercept_coords[0] = 3
                            order =  [x_intercept_coords, y_intercept_coords]
                        else:
                            # y intercept
                            y_intercept_coords[0] = 0
                            order =  [y_intercept_coords, x_intercept_coords]
                    
                
                lines_to_fill.append([s_pose_coords[1], order[0][1], s_pose_coords[0]]) # 
                lines_to_fill.append([order[0][1], order[1][1], order[0][0]])
                lines_to_fill.append([order[1][1], matrix_position, position_square])
        
        else:
            print("WARNING: LINE DETECTION FAILED. CONSIDER VERIFYING ERROR. CODE: 5")
        
        return self.fill_lines(lines_to_fill)


    def apply_square_multipliers(self, intercept):
        '''
        The four squares with their respetive locations
                        1  |  0
                        -------
                        3  |  2
        '''
        s = intercept[0]
        new_point = [intercept[1][0], intercept[1][1]]
        if s == 0:
            return new_point
        if s == 1:
            new_point[0] *= -1
            return new_point
        if s == 2:
            new_point[1] *= -1
            return new_point
        if s == 3:
            new_point[0] *= -1
            new_point[1] *= -1
            return new_point
        
    def dtr(self, a):
        return math.pi*a/180

    def insert_value_map(self, square_number, matrix_position, value, pose_update=False):
        '''
        inserts the value given, into the position specified by position and square
        Value table: 
            0: not discovered yet
            1: obstruction
            2: free space
            3: past or present pose of the robot
            4: 
        '''

        if pose_update or self.map[square_number][matrix_position[1]][matrix_position[0]] == 0:
            # we only update if its a pose update, or  values that have nothing assigned to them yet
            self.map[square_number][matrix_position[1]][matrix_position[0]] = value
        
    
    def verify_distance(self, s, d):
        point_value = 1
        if d == self.sensors[s].default_value:
            d = self.sensors[s].max
            point_value = 2
        if d < 0:
            return
        return d, point_value
    
    def get_sensor_aware_pose(self, sensor):
        print("self.pose[2]")
        print(self.pose[2])
        rotation_matrix = np.array(
            [
                [math.cos(-self.pose[2]), -math.sin(-self.pose[2])],
                [math.sin(-self.pose[2]), math.cos(-self.pose[2])]
            ])

        point = np.array([self.sensors[sensor].pose[0], self.sensors[sensor].pose[1]])
        new_point = rotation_matrix@point
        
        
        return [new_point[0]+self.pose[0], new_point[1]+self.pose[1], (self.sensors[sensor].pose[2] + self.pose[2])%(2*math.pi)]
    
    # MAIN METHOD: ALL OTHER FUNCTIONS ACT AS SUPPORTING FUNCTION
    def insert_sensor_point(self, sensor, distance):
        point_value = 1 # self.verify_distance(sensor, distance)
        if not distance:
            return

        modified_pose = self.get_sensor_aware_pose(sensor)
        pose_coords = self.get_matrix_coords(modified_pose)

        # absolute position of detected point from the sensor
        absolute_position = self.get_absolute_position(modified_pose, distance)
        square_number, matrix_position = self.get_matrix_coords(absolute_position)

        # resizes the matrix according to where the sensors have detected something/nothing
        # note: resize every square to the maximum distance from 0,0
        self.verify_matrix_sizes(square_number, matrix_position)
        
        self.insert_value_map(square_number, matrix_position, point_value)
        
        self.fill_in_line_of_sight(square_number, matrix_position, absolute_position, [pose_coords, modified_pose])
        
    # MAIN METHOD: ALL OTHER FUNCTIONS ACT AS SUPPORTING FUNCTION
    def update_matrix_map(self, distance, value):
        '''
        Given the current pose (with yaw), and the distance that a sensor has received,
        updates that data point in the localization matrix
        '''
        
        if distance < 0:
            return
        
        # let's modfy the pose, correcting for location and angle of the sensor
        
        absolute_position = self.get_absolute_position(self.pose, distance)
        square_number, matrix_position = self.get_matrix_coords(absolute_position)
        
        # resizes the matrix according to where the sensors have detected something/nothing
        # note: resize every square to the maximum distance from 0,0
        self.verify_matrix_sizes(square_number, matrix_position)
        
        self.insert_value_map(square_number, matrix_position, value)
        
        self.fill_in_line_of_sight(square_number, matrix_position, absolute_position, 0)

    def reverse_columns(self, matrix):
        '''reverses the order of columns. Used for square manipulation'''
        draft = []
        for i in range(len(matrix)):
            draft.append(matrix[i][::-1])
        return draft
    
    def reverse_rows(self, matrix):
        '''reverses the order of rows. Used for square manipulation'''
        draft = []
        for i in range(len(matrix)-1, -1, -1):
            draft.append(matrix[i])
        return draft
    
    def normalize_columns_and_columns(self):
        '''takes the existing data, and normalizes it, so that all rows are the same length, as well as all columns'''
        
        for square in range(4): 
            if len(self.map[square]) != self.max_dim:
                for i in range(self.max_dim - len(self.map[square])):
                    self.map[square].append([])
            for r in range(len(self.map[square])):
                if len(self.map[square][r]) != self.max_dim:
                    for i in range(self.max_dim-len(self.map[square][r])):
                        self.map[square][r].append(0)

    def print_map(self, index):
        '''Normalizes the current map, and prints out a '''
        
        
        self.normalize_columns_and_columns()
        
        top_right = self.reverse_rows(self.top_right)
        top_left = self.reverse_rows(self.reverse_columns(self.top_left))
        
        bottom_right = self.bottom_right
        bottom_left = self.reverse_columns(self.bottom_left)
        
        hemispheres = [[top_left, top_right], [bottom_left, bottom_right]]
        ran = False
        final_map = ""
        self.matrix_map = []

        for h in hemispheres:
            for r in range(len(top_left)):
                row = []
                for s in range(2):
                    for c in range(len(h[s][r])):
                        char = " " if str(h[s][r][c]) == "0" else str(h[s][r][c])
                        final_map +=   char + " "
                        row.append(int(h[s][r][c]))
                    if s != 1:
                        final_map +=  "a "
                final_map += "\n"
                self.matrix_map.append(row)
            if not ran:
                ran = True
                for i in range(2*len(top_left)+1):
                    final_map += "a "
                final_map += '\n'
                
        self.map_print = final_map

        Printing.view_map(self.matrix_map, index)

class Helpers:
    def __init__():
        pass
    
    # This static method has been copy pasted from here:
    # https://iqcode.com/code/python/python-bresenham-line-algorithm
    @staticmethod
    def get_line_bresenham(start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end
    
        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        """
        
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
    
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
    
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
    
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
    
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
    
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
    
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
    
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points
       

from PIL import Image
import numpy as np
class Printing:
    def __init__():
        pass
    @staticmethod
    def get_color(v):
        # Value table: 
        #     0: not discovered yet
        #     1: obstruction
        #     2: free space
        #     3: past or present pose of the robot
        if v == 0:
            return (0, 0, 0)
        if v == 1:
            return (255, 0, 0)
        if v == 2:
            return (0, 0, 255)
        if v == 3:
            return (0, 255, 0)
    @staticmethod
    def view_map(matrix, index, type=1):
        if type == 0:
            pass #print(np.array(matrix))
        else:
            matrix = np.array(matrix).T
            # taken from here: https://stackoverflow.com/questions/20304438/how-can-i-use-the-python-imaging-library-to-create-a-bitmap
            img = Image.new('RGB', (len(matrix),len(matrix[0])), "Black") # Create a new black image
            pixels = img.load() # Create the pixel map
            for i in range(img.size[0]):    # For every pixel:
                for j in range(img.size[1]):
                    
                    pixels[i,j] = Printing.get_color(matrix[i][j]) # (v, v, v) # Set the colour accordingly
            
            img.save('Maze'+str(index)+'.bmp')
