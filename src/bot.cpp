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
 * @file bot.cpp
 * @author Charu Sharma (charu107@umd.edu)
 * @brief to implement obstacle avoidance behavior to turtlbot
 * @version 0.1
 * @date 2021-11-28
 */


#include <iostream>
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "turtlebot-simulations/bot.hpp"
#include "sensor_msgs/LaserScan.h"


/**
 * @brief      Constructs the object.
 */

Bot::Bot(ros::NodeHandle n) {
ROS_INFO("Creating the walker behaviour...");

lidar_data = n.subscribe < sensor_msgs::LaserScan
    > ("/scan", 10, &Bot::readLidar, this);
}

void Bot::turtlebotInitiate(ros::NodeHandle n,
                            ros::Publisher chatter_pub,
                            ros::Rate loop_rate) {
  geometry_msgs::Twist msg;

  // Keep running till ros is running fine
  while (ros::ok()) {
      // Check for obstacle
      if (obstacle_detected) {
        // Obstacle encountered
        stopRobot(chatter_pub, loop_rate);  // Stop the robot
        ROS_WARN_STREAM("OBSTACLE DETECTED! Computing alternate path...");
        keepTurning(n, chatter_pub, loop_rate);
    } else {
        ROS_INFO_STREAM("MOVING FORWARDS!");
        msg.linear.x = front_speed;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;
    }
    // Publish the twist message to anyone listening
    chatter_pub.publish(msg);

    // "Spin" a callback in case we set up any callbacks
    ros::spinOnce();

    // Sleep for the remaining time until we hit our 10 Hz rate
    loop_rate.sleep();
  }
}


/**
 * @brief      Avoids the obstacle
 */
void Bot::keepTurning(ros::NodeHandle n, ros::Publisher chatter_pub,
                             ros::Rate loop_rate) {
  // Keep the turtlebot turning
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = turn_speed;

  // Publish the twist message to anyone listening
  chatter_pub.publish(msg);
  while (!path_clear) {
    // Sleep for the remaining time until we hit our 10 Hz rate
    loop_rate.sleep();

    // "Spin" a callback in case we set up any callbacks
    ros::spinOnce();
  }
  stopRobot(chatter_pub, loop_rate);
  // Returns the collision flag
  obstacle_detected = false;
}

/**
 * @brief      to stop the turtlebot
 */
void Bot::stopRobot(ros::Publisher chatter_pub, ros::Rate loop_rate) {
  geometry_msgs::Twist msg;
  // Stop the turtlebot
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  // Publish the twist message to anyone listening
  chatter_pub.publish(msg);
  ros::Duration(sleep_duration).sleep();
}

/**
 * @brief      Callback for the laser scan data
 *
 * @param[in]  msg   The message
 */
void Bot::readLidar(const sensor_msgs::LaserScan::ConstPtr &msg) {
  int lidar_range = 30;

  std::vector<double> obstacles_detected;
  std::vector<double> lidar_range_vect;

  lidar_range_vect = std::vector<double>(msg->ranges.begin(),
                                         msg->ranges.begin() + lidar_range);
  obstacles_detected = std::vector<double>(msg->ranges.end() - lidar_range,
                                           msg->ranges.end());

  obstacles_detected.insert(obstacles_detected.end(), lidar_range_vect.begin(),
                            lidar_range_vect.end());

  path_clear = true;

  for (auto &range : obstacles_detected) {
    if (range < collision_threshold) {
      // Returns the collision flag
      obstacle_detected = true;
      path_clear = false;
    }
  }
}
