/************************************************************************
MIT License
Copyright © 2021 Charu Sharma
Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the “Software”), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included 
in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************/

/**
 * @file bot.hpp
 * @author Charu Sharma (charu107@umd.edu)
 * @brief to implement walker behavior to turtlbot
 * @version 0.1
 * @date 2021-11-28
 * This program will run the walker node for the turtlebot
 */

#ifndef INCLUDE_TURTLEBOT_SIMULATIONS_BOT_HPP_
#define TURTLEBOT_SIMULATIONS_BOT_HPP_
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief      Class for Bot.
 */
class Bot {
 private:
  // Declaring the front speed of the turtlebot
  double front_speed = 0.2;
  // Declaring the turning speed of the turtlebot
  double turn_speed = 0.5;
  // Subscriber for the LIDAR Data
  ros::Subscriber lidar_data;
  // boolean for obstacle detection
  bool obstacle_detected = false;
  // boolean for making sure the robot can move ahead
  bool path_clear = true;
  // declaring a minimum distance the robot can stay away from the obstacle
  double collision_threshold = 0.35;
  // sleep duration so that stop doesn't keep getting called
  double sleep_duration = 0.4;

 public:
  // Declaring the new constructor
  explicit Bot(ros::NodeHandle);
  // Function to start the turtlebot motion in the environment
  void turtlebotInitiate(ros::NodeHandle, ros::Publisher, ros::Rate);
  // Function to keep turning while the robot is clear of any obstacles
  void keepTurning(ros::NodeHandle, ros::Publisher, ros::Rate);
  // Function to stop the robot
  void stopRobot(ros::Publisher, ros::Rate);
  // get LIDAR Data
  void readLidar(const sensor_msgs::LaserScan::ConstPtr&);
};
#endif  // TURTLEBOT_SIMULATIONS_BOT_HPP_
