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

Then install these dependencies:
```
sudo apt install ros-noetic-costmap-2d -y
sudo apt install liborocos-bfl-dev
sudo apt-get install ros-noetic-nav-core
sudo apt-get install ros-noetic-navfn
sudo apt-get install ros-noetic-kobuki-msgs
```
and after that 
```
catkin_make
```
