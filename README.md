# TurtleBot-simulation
This project implements a roomba type behaviour on the commonly used turtlebot platform in ROS. It has a launch file demo.launch which launches the turtlebot gazebo simulation and also launches the custom bot node. It also has a custom gazebo world, where the turtlebot performs.

ROS publisher/subscriber beginner tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-Default.svg)](https://opensource.org/licenses/MIT)


## Authors
[Charu Sharma](<https://github.com/Sharma117555448>) (UID 117555448)

# Dependencies
Ubuntu 18.04

ROS Melodic 

Turtlebot packages

Modern C++ Programming Language

# Build
## Steps to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Sharma117555448/turtlebot-simulations.git
cd ~/catkin_ws/
catkin_make
```
# Turtlebot3 installation:
Follow the instructions to install the Turtlebot3 ROS package 
```
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
Kill the existiing Gazebo servers
```
killall gzserver
```
Set the environment variable for the turtlebot's model
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```
# Run
## Steps to run
## Launch all nodes individually
## 1. Run roscore
```
cd ~/catkin_ws/
source ./devel/setup.bash
roscore
```
## 2. Run roslaunch without rosbag recording
```
cd ~/catkin_ws
source devel/setup.bash
catkin_make
roslaunch turtlebot-simulations demo.launch
```
## 3. Run roslaunch without rosbag recording
The records are saved in results as rosbag.bag
```
cd ~/catkin_ws
source devel/setup.bash
catkin_make
roslaunch turtlebot-simulations demo.launch nable_ros_bag:=true
```
## 4. To examine the rosbag
```
cd results
rosbag info rosbag.bag
```
## 5. To play the rosbag
```
cd results 
rosbag play rosbag.bag
```
# Run cppcheck
Results are stored in `./results/cppcheck.txt` 
```
cppcheck --enable=all --std=c++11 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./lib") > results/cppcheck.txt 2>&1
```
# Run cpplint
Results are stored in `./results/cpplint.txt`
```
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" -e "^./lib/") > results/cpplint.txt 2>&1
```
# Reference
http://wiki.ros.org/