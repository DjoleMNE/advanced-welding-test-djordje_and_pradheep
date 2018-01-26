/*
Created on: December, 2017
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
#include <ros/console.h>
#include <sstream>
#include <Eigen/Dense>
#include <awt_trajectory_generator.h>
#include "visualization_msgs/Marker.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "awt_trajectory_generator_node");
    std::string type, side;
    double phase = 0;
    double amplitude = 0;
    int num_steps = 0, side_gain = 1, cycles = 1;
    double time_step = 0, loop_freq = 0, x_unit_gain = 0, y_unit_gain = 0;
    Eigen::MatrixXd velocity_matrix;
    std::string root_frame = "arm_link_0";

    ros::NodeHandle node_handle("~");
    node_handle.getParam("type", type);
    node_handle.getParam("phase", phase);
    node_handle.getParam("amplitude", amplitude);
    node_handle.getParam("num_steps", num_steps);
    node_handle.getParam("time_step", time_step);
    node_handle.getParam("loop_freq", loop_freq);
    node_handle.getParam("x_unit_gain", x_unit_gain);
    node_handle.getParam("y_unit_gain", y_unit_gain);
    node_handle.getParam("side", side);
    node_handle.getParam("cycles", cycles);

    geometry_msgs::TwistStamped velocity;

    ros::Publisher velocity_publisher = \
    node_handle.advertise<geometry_msgs::TwistStamped>("/arm_1/arm_controller/cartesian_velocity_command", 100);

    ros::Publisher marker_pub = \
    node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    traj_gen::AWT_Trajectory_Generator generator;

    double step_size_x;
    Eigen::MatrixXd wave;

    if (type == "sine"){
        generator.generate_sine_wave(
                    phase,
                    amplitude,
                    num_steps,
                    wave,
                    step_size_x);
    }

    else if (type == "square" ){
        generator.generate_square_wave(
                    phase,
                    amplitude,
                    num_steps,
                    wave,
                    step_size_x);
    }


    generator.generate_trajectory(type,
                                phase,
                                amplitude,
                                num_steps,
                                time_step,
                                velocity_matrix);
    if (side == "right") side_gain = -1;
    else side_gain = 1;

    ROS_INFO_STREAM("The marker publisher need some time to connect to subscribers.");
    while(!marker_pub.getNumSubscribers() > 0){}
    ROS_INFO_STREAM("The marker publisher has connected to subscribers.");

    ROS_INFO_STREAM("The velocity publisher need some time to connect to subscribers.");
    while(!velocity_publisher.getNumSubscribers() > 0){}
    ROS_INFO_STREAM("The velocity publisher has connected to subscribers.");


    ros::Rate loop_rate(loop_freq);
    velocity.header.frame_id = root_frame;
    int step_counter = 0;
    visualization_msgs::Marker marker;
    marker.id = 0;

    for (int i = 0; i < velocity_matrix.rows()*cycles; i++)
    {
        if (step_counter == velocity_matrix.rows()) step_counter = 0;
        marker.header.frame_id = "arm_link_5";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (wave(step_counter, 1)/x_unit_gain)*0.05;
        marker.pose.position.y = (side_gain * wave(step_counter, 0)/y_unit_gain)*0.1;
        marker.pose.position.z = 0.0;
        std::cout << "Marker" << '\n';
        std::cout<<marker.pose.position.x<<"\n";
        std::cout<<marker.pose.position.y<<"\n";
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.01;
        marker.scale.z = 0.008;
        marker.color.r = 0.0;
        marker.color.g = 2.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker.id+=1;
        step_counter++;
        marker_pub.publish(marker);
        loop_rate.sleep();
    }

    step_counter = 0;
    for (int i = 0;  i < velocity_matrix.rows() * cycles; i++)
    {
        if (step_counter == velocity_matrix.rows()) step_counter = 0;
        velocity.twist.linear.x = velocity_matrix(step_counter, 1)/x_unit_gain;
        velocity.twist.linear.y = side_gain * velocity_matrix(step_counter, 0)/y_unit_gain;
        step_counter++;
        velocity_publisher.publish(velocity);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO_STREAM("The trajectory has been published!");

 return 0;
}
