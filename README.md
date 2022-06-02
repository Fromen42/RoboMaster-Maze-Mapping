# RoboMaster-Maze-Mapping

Here is a ROS package that implements the RoboMaster-Maze-Mapping

## Installation

Unzip the package and copy the root folder to the source folder of your workspace, `~/dev_ws/src`:

```shell
$ unzip Robomaster_Maze.zip
$ mv Robomaster_Maze ~/dev_ws/src
```

Build your workspace, so that the new package becomes available

```shell
$ cd ~/dev_ws
$ colcon build
```

## Usage

Open CoppeliaSim:

```shell
$ ~/apps/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/coppeliaSim.sh
```

Load the first world, `Maze.ttt` or `Maze.withdoor.ttt` or `room.ttt` or `Test.ttt`, from the Coppelia GUI, enable real-time mode and start the simulation.

Open a second terminal and start the Robomaster ROS bridge:

```shell
$ source ~/dev_ws/install/setup.bash
$ ros2 launch robomaster_ros ep.launch name:=robo1 tof_0:=True tof_1:=True tof_2:=True tof_3:=True
```

Open a third terminal, source your workspace and launch `align.launch.py`, which implements the second and third tasks: 

```shell
$ source ~/dev_ws/install/setup.bash
$ colcon build --packages-select Robomaster_Maze
$ ros2 launch Robomaster_Maze controller.launch.py robomaster_name:=robo1
```

While running the program it creates a Maze0.bmp file and updates it everytime there is an update to the map and it can be found at ~/dev_ws



### Screenshots

![Maze](Mapping-Outputs/test0 4.png?raw=true "Title")


