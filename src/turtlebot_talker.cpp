
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
 * @file main.cpp
 * @author Charu Sharma (charu107@umd.edu)
 * @brief main to implement walker behavior to turtlbot
 * @version 0.1
 * @date 2021-11-28
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <memory>
#include <sstream>
#include "turtlebot-simulations/bot.hpp"

/**
 * @brief      main function
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     nothing
 */
int main(int argc, char* argv[]) {
  // Initialize the ros node
  ros::init(argc, argv, "bot");
  ros::NodeHandle nh;

// Declaring the Twist publisher
  ros::Publisher chatter_pub = nh.advertise < geometry_msgs::Twist
                                                  > ("cmd_vel", 1000);

  // Set up the publisher rate to 10 Hz
  ros::Rate loop_rate(10);
  // Declaring a controller
  Bot bot(nh);
  // start the roomba like turtlebot
  bot.turtlebotInitiate(nh, chatter_pub, loop_rate);

  return 0;
}