# human_aware_navigation
This repository contains ROS packages designed to enable navigation path planning that accounts for human comfort. In this forked version, I have resolved dependency issues and updated the lab configurations to ensure compatibility with ROS Noetic (version 22.01).

If you use this code in your research, please consider citing the original paper by Kollmitz et al.

# Here is the installation steps for the noetic devel :
cd ~/catkin_ws/src

Clone the following packages from GitHub:
```
for noetic: 
$git clone -b noetic-devel https://github.com/MahboubeHp/people.git
$git clone -b noetic-devel https://github.com/MahboubeHp/lattice_planner.git
$git clone -b noetic-devel https://github.com/MahboubeHp/human_aware_navigation.git
$git clone -b noetic-devel https://github.com/MahboubeHp/timed_path_follower.git
$git clone -b noetic https://github.com/MahboubeHp/wu_ros_tools.git
```

Then install the dependencies below, you may need to install some of them locally within your catkin_ws:
```
$sudo apt install ros-noetic-costmap-2d -y
$sudo apt install liborocos-bfl-dev
$sudo apt-get install ros-noetic-nav-core
$sudo apt-get install ros-noetic-navfn
$sudo apt-get install ros-noetic-kobuki-msgs
```
```
$catkin_make
$source devel/setup.bash
```
If you wish to test the Human aware navigation with Turtlebot3 consider installing it 
```
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations

```
Otherwise consider bringing up your desired robot and test it. Please refer to the Human aware navigation wiki by Marina Kollmitz for the information on how to controll the robot using move_base.

You might notice that the Turtlebot3 moves randomly by itself. looking into the topics, you can observe that the Turtlebot3_drive node in addition to the move_base is publishing into /cmd_vel. killing this node directly will stop gazebo.

A solution to fix this issue is to navigate to the turtlebot3_gazebo package which can be located in: 

../turtlebot3_gazebo/launch/turtlebot3_simulation.launch 
then, Commenting out the turtlebot3_drive node to prevent it from starting. You can comment out the line below to fix the behaviour:
```
<node name="$(arg name)_drive" pkg="turtlebot3_gazebo" type="turtlebot3_drive" required="true" output="screen"/> 
```
