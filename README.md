# RoboMaster-Maze-Mapping

ros2 launch Robomaster_Maze controller.launch.py robomaster_name:=robo1


source ~/dev_ws/install/setup.bash


colcon build --packages-select Robomaster_Maze


ros2 launch robomaster_ros ep.launch name:=robo1 tof_0:=True tof_1:=True tof_2:=True tof_3:=True


~/apps/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/coppeliaSim.sh
