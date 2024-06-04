# human_aware_navigation
The human_aware_navigation repository includes ROS packages to enable the planning of navigation paths that take human comfort into account 

If you use the code for your research, please consider citing the paper:

```
@INPROCEEDINGS{kollmitz15ecmr,
  author = {Marina Kollmitz and Kaijen Hsiao and Johannes Gaa and Wolfram Burgard},
  title = {Time Dependent Planning on a Layered Social Cost Map for Human-Aware Robot Navigation},
  booktitle = {Proc.~of the IEEE Eur.~Conf.~on Mobile Robotics (ECMR)},
  year = {2015},
  doi = {10.1109/ECMR.2015.7324184},
  url = {http://ais.informatik.uni-freiburg.de/publications/papers/kollmitz15ecmr.pdf}
}
```

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
and after that 
```
$catkin_make
$source devel/setup.bash
```
If you wish to test the Human aware navigation with Turtlebot3 consider installing it 
```
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations

```
Otherwise consider bringing up your fancy robot and test it. Please refer to the Human aware navigation wiki by Marina Kollmitz for the information on how to controll the robot using move_base.

You might notice that the Turtlebot3 moves randomly by itself. looking into the topics, you can notice that the Turtlebot3_drive node in addition to the move_base is publishing into /cmd_vel. killing this node directly will stop gazebo.

A solution to fix this issue is killing this node is to navigate to the turtlebot3_gazebo package like: 

../turtlebot3_gazebo/launch/turtlebot3_simulation.launch and then, Commented out the turtlebot3_drive node to prevent it from starting as the lines below:

  <!-- <node name="$(arg name)_drive" pkg="turtlebot3_gazebo" type="turtlebot3_drive" required="true" output="screen"/> -->
