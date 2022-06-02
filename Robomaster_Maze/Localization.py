import math
import os
from re import M
import time
import numpy as np
from Helpers import Helpers
from Printing import Printing
from colorama import init
from colorama import Fore

class Sensor:
    def __init__(self, name, sensor_pose, max_distance, default_value=-1):
        self.name = name
        self.pose = sensor_pose
        self.max_distance = max_distance # the maximum distance that the sensor can sense
        self.default_value = default_value # the value that is returned when it finds nothing
        
class Localization:
    def __init__(self, sensors, precision=0.05):
        '''
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
        return (p2[1] - p1[1]) / (p2[0] - p1[0])
    
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
        # print('custom')
        # print(s_pose_coords)
        # print(s_pose)
        # print("checking")
        # print(self.pose_coords)
        # print(self.pose)
        # exit()
        if s_pose_coords[0] == position_square:
            print("case 0")
            lines_to_fill.append([s_pose_coords[1], matrix_position, s_pose_coords[0]]) 
            # exit()
            return self.fill_lines(lines_to_fill)
        
        x_intercept = None
        y_intercept = None
        
        
        if abs(s_pose_coords[0] - position_square) == 2:
            print("case 1")
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
            print("case 2")
            # one position is right, one is left: Line goes horizontal (but not diagonal!). There is a slope
            y_intercept = self.get_y_intercept(s_pose[:2], absolute_position)
            intercept_coords = self.get_matrix_coords(y_intercept)
            
            lines_to_fill.append([s_pose_coords[1], intercept_coords[1], s_pose_coords[0]]) # 
            lines_to_fill.append([intercept_coords[1], matrix_position, position_square])
             
        elif abs(s_pose_coords[0] + position_square) == 3:
            print("case 3")
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
                if slope_points < 0:
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
                        # in 0
                        # x_intercept_coords[0] = 
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
                    
                    
                    '''
                        1  |  0
                        -------
                        3  |  2
                    '''
                
                print()
                lines_to_fill.append([s_pose_coords[1], order[0][1], s_pose_coords[0]]) # 
                lines_to_fill.append([order[0][1], order[1][1], order[0][0]])
                lines_to_fill.append([order[1][1], matrix_position, position_square])
        
        
        else:
            print("WARNING: LINE DETECTION FAILED. CONSIDER VERIFYING ERROR. CODE: 5")
        
        return self.fill_lines(lines_to_fill)

    def apply_square_multipliers(self, intercept):
        '''
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
        # print("LALALA")
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
        
        # print(sensor)
        point = np.array([self.sensors[sensor].pose[0], self.sensors[sensor].pose[1]])
        # print(point)
        new_point = rotation_matrix@point
        
        
        return [new_point[0]+self.pose[0], new_point[1]+self.pose[1], (self.sensors[sensor].pose[2] + self.pose[2])%(2*math.pi)]
    
    # MAIN METHOD: ALL OTHER FUNCTIONS ACT AS SUPPORTING FUNCTION
    def insert_sensor_point(self, sensor, distance):
        distance, point_value = self.verify_distance(sensor, distance)
        if not distance:
            return
        # print(self.pose)
        modified_pose = self.get_sensor_aware_pose(sensor)
        # print("md_p")
        # print(modified_pose)
        # exit()
        pose_coords = self.get_matrix_coords(modified_pose)

        
        
        # absolute position of detected point from the sensor
        absolute_position = self.get_absolute_position(modified_pose, distance)
        # print(distance)
        # print("tada?")
        # print(absolute_position)
        # print("The Two")
        # exit()
        square_number, matrix_position = self.get_matrix_coords(absolute_position)
        # print(square_number)
        # print(matrix_position)
        # exit()
        # resizes the matrix according to where the sensors have detected something/nothing
        # note: resize every square to the maximum distance from 0,0
        self.verify_matrix_sizes(square_number, matrix_position)
        
        self.insert_value_map(square_number, matrix_position, point_value)
        
        
        # absolute_position_pose = self.get_absolute_position(modified_pose, 0.001)
        # sensor_coords = self.get_matrix_coords(absolute_position_pose)
        # self.insert_value_map(sensor_coords[0], sensor_coords[1], 5)
        print(modified_pose)
        print(square_number, matrix_position)
        print()
        
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
        
        self.fill_in_line_of_sight(square_number, matrix_position, absolute_position)

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

    def print_map(self):
        '''Normalizes the current map, and prints out a '''
        init()
        
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
                        color = Fore.BLACK if str(h[s][r][c]) == "0" else Fore.GREEN
                        char = " " if str(h[s][r][c]) == "0" else str(h[s][r][c])
                        final_map +=  color + char + " "
                        row.append(int(h[s][r][c]))
                    if s != 1:
                        final_map += Fore.BLUE + "a "
                final_map += "\n"
                self.matrix_map.append(row)
            if not ran:
                ran = True
                for i in range(2*len(top_left)+1):
                    final_map += Fore.BLUE + "a "
                final_map += '\n'
                
        self.map_print = final_map
        print(np.array(self.matrix_map))
        Printing.view_map(self.matrix_map)