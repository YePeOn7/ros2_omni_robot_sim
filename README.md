# Omnidirectional Robot Gazebo Simulation

## Introduction

This repository contains a Gazebo Ignition simulation that can be run with ROS 2, enabling the simulation of 3 to 6 wheeled omnidirectional robots. This setup is particularly useful for robotics research and simulation, allowing for the study and testing of complex robot behaviors and control strategies.

### Tested on

- Ubuntu 22.04
- ROS Humble
- Gazebo Ignition Fortres

## Available Robot Model

![3W Robot Control](gif/3w.gif)  
*3 wheel omni robot (3w)*

![3W Robot Control v2](gif/3w_v2.gif)  
*3 wheel omni robot version 2 (3w_v2) -> default*

![4W Robot Control](gif/4w.gif)  
*4 wheel omni robot (4w)*

![5W Robot Control](gif/5w.gif)  
*5 wheel omni robot (5w)*

![6W Robot Control](gif/6w.gif)  
*6 wheel omni robot (6w)*

## How to Use This Repository

Please clone this repository to your ros2 workspace. The following assume your ros2 work space placed on your home directory with named **ros2_ws**

```bash
cd ~/ros2_ws/src
git clone git@github.com:YePeOn7/ros2_omni_robot_sim.git
```

Install dependency by running the following command

```bash
cd ~/ros2_ws/ros2_omni_robot_sim
sudo chmod +x install_dependency.sh
./install_dependency.sh
```

build the package by using the following command

```bash
colcon build
source install/setup.bash
```

Determine the robot Model by setting the environment variable of **OMNI_ROBOT_MODEL** with the robot model listed on [Available Robot Model](#available-robot-model). For example, if you want to use model of **3 wheel omni robot version 2**, you can set as follow:

```bash
export OMNI_ROBOT_MODEL=3w_v2
```

After that you can run several of the following simulations

### 1. Manual Teleop

```bash
ros2 launch ros2_omni_robot_sim gazebo_sim.launch.py
```

or you can specity the world as follow

```bash
ros2 launch ros2_omni_robot_sim gazebo_sim.launch.py world:=<world_name>
```

*please refer to [World Options](#world-options) Section for <world_name>*

![Robot Control](gif/control.gif)

### 2. SLAM with slam_toolbox

Start the SLAM by using the following command

```bash
ros2 launch ros2_omni_robot_sim slam_gazebo_sim.launch.py

# with specific world
ros2 launch ros2_omni_robot_sim slam_gazebo_sim.launch.py world:=<world_name>
```

*please refer to [World Options](#world-options) Section for <world_name>*

![Robot Control](gif/slam.gif)

Save the map by using the following command

```bash
cd ~/ros2_ws/ros2_omni_robot_sim/map # You can use other folder, however you need reconfigure the launch file to use your map
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

**Recommendation**: It is advised to use the **world name** as the **`<map_name>`** when saving your map. This ensures the map is automatically picked up by the launch file without additional configuration.

- For example: If your world is named **`maze1`**, name your map **`maze1`** as well.
- Saved files will be: `maze1.pgm` and `maze1.yaml`.

This consistency simplifies navigation setup and avoids manual path adjustments in your ROS 2 launch files.

### 3. Navigation

Start the SLAM by using the following command

```bash
ros2 launch ros2_omni_robot_sim navigation_gazebo_sim.launch.py

# with specific world
ros2 launch ros2_omni_robot_sim navigation_gazebo_sim.launch.py world:=<world_name>
```

*please refer to [World Options](#world-options) Section for <world_name>*

![Robot Control](gif/navigation.gif)

## Demo Video

[![Demo Video](https://img.youtube.com/vi/WKV7GRxiQOE/maxresdefault.jpg)](https://youtu.be/WKV7GRxiQOE)

## World Options

- maze1
- maze2 (default)
