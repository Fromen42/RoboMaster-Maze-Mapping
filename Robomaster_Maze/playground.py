from Localization import Localization
import math
from Helpers import Helpers 
from Localization import Sensor
import os

os.system('clear')
def dtr(a):
    return math.pi*a/180

sensor1 = Sensor('front', (4, 4, dtr(0)), 10)
sensor2 = Sensor('right', (4, 4, dtr(90)), 10)
sensor3 = Sensor('left', (-3, -3, dtr(60)), 10)
#  def __init__(self, name, sensor_pose, max_distance, default_value=-1)
map = Localization([sensor1, sensor2, sensor3], 0.1)

LiDAR_points = [4, 6, 2, 4, 5, 3.12]
poses = [[0, 0, 0], [0, 0, dtr(180)], [0, 0, dtr(90)], [0, 0, dtr(45)], [0, 0, 0], [0, 0, 0]]

# LiDAR_points = [2, 2, 8]
# poses = [[0, 0, 0], [-1, -1, dtr(220)], [2, -2, dtr(270)]]
# LiDAR_points = [2, 2, 2, 2]
# poses = [[-4, -3, dtr(-3330)], [3, -3, dtr(0)], [-2, 3, dtr(0)], [1, 3, dtr(0)]]
LiDAR_points = [10]
# poses = [(0, 0, dtr(30)), (3, 3, dtr(250)), (-2, -2, dtr(60))]
poses = [(0, 0, dtr(30))]


dirs = ["front", "right", "left"]
# print("start")
# map.update_current_pose([0, 0, dtr(0)])
# map.insert_sensor_point("front", 10)
# map.insert_sensor_point("right", 10)
# map.update_current_pose([0, 0, dtr(45)])
# map.insert_sensor_point("front", 10)
# map.insert_sensor_point("right", 10)
# # map.update_current_pose([0, 0, dtr(65)])
# # map.insert_sensor_point("front", 10)
# # map.insert_sensor_point("right", 10)

# print("printing map")
# map.print_map()
      
# exit()
     
for pose in poses:
    for i in range(10):
        angle = i*math.pi/30
        for j in range(len(["front", "right", "left"])):
            map.update_current_pose([pose[0], pose[1], angle])
            map.insert_sensor_point(dirs[j], 10)

# map.update_current_pose([-3, -6, 3*math.pi/30])      
# map.insert_sensor_point("front", 10)
# map.update_current_pose([0, 0, 0])     
# map.update_matrix_map(0.01, 4)
# map.update_matrix_map(1, 4)
# map.update_matrix_map(2, 4)
# map.update_matrix_map(3, 4)
# map.update_matrix_map(4, 4)
# map.update_matrix_map(5, 4)
# for i in range(10):
#     angle = i*math.pi/30
#     for j in range(len(["front", "right", "left"])):
#         map.update_current_pose([5, 5, angle])
#         map.insert_sensor_point(dirs[j], 10)
        
        # map.update_matrix_map(LiDAR_points[i], 1)
print("printing map")
map.print_map()
# parts = 100
# inc = 360/100
# for i in range(1, parts):
#     map.update_matrix_map([0,0,dtr(i*inc)], 5, 1)
#     if i == 100:
#         break
'''
for i in range(len(LiDAR_points)):
    map.insert_sensor_point("front", LiDAR_points[1])
    map.update_matrix_map(LiDAR_points[i], 1)
    map.print_map()
'''
    # if i == 2:
    #     exit()
    

# map.create_bitmap()

# matrix = [[1,2], [3,4]]
# nm = []

    
# print(nm)