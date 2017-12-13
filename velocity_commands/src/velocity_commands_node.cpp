/*
Created on: Dec 4, 2017
Author(s): Djordje Vukcevic, Pradheep Padmanabhan
Copyright (c) [2017]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <kdl/kdl.hpp>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_commands_node");
    double linear_x = 0;
    double linear_y = 0;
    double linear_z = 0;
    double angular_x = 0;
    double angular_y = 0;
    double angular_z = 0;
    std::string frame = "arm_link_0";

    ros::NodeHandle node_handle("~");
    node_handle.getParam("linear_x", linear_x);
    node_handle.getParam("linear_y", linear_y);
    node_handle.getParam("linear_z", linear_z);
    node_handle.getParam("angular_x", angular_x);
    node_handle.getParam("angular_y", angular_y);
    node_handle.getParam("angular_z", angular_z);
    node_handle.getParam("root_name", frame);

    geometry_msgs::TwistStamped velocity;

    ros::Publisher velocity_publisher = \
    node_handle.advertise<geometry_msgs::TwistStamped>("/arm_1/arm_controller/cartesian_velocity_command", 1);

    ros::Rate loop_rate(50);
    velocity.header.frame_id = frame;
    velocity.twist.linear.x = linear_x;
    velocity.twist.linear.y = linear_y;
    velocity.twist.linear.z = linear_z;
    velocity.twist.angular.x = angular_x;
    velocity.twist.angular.y = angular_y;
    velocity.twist.angular.z = angular_z;

    while (ros::ok())
    {
    velocity_publisher.publish(velocity);
    ros::spinOnce();
    loop_rate.sleep();
    }
 return 0;
}
