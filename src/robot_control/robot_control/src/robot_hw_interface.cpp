/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Robot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <robot_control/robot_hw_interface.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

namespace robot_control
{

RobotHWInterface::RobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{

  ros::NodeHandle n;

  lvelfront_pub = n.advertise<std_msgs::Float32>("/lvelfront", 1000);
  lvelcenter_pub = n.advertise<std_msgs::Float32>("/lvelcenter", 1000);
  lvelrear_pub = n.advertise<std_msgs::Float32>("/lvelrear", 1000);
  rvelfront_pub = n.advertise<std_msgs::Float32>("/rvelfront", 1000);
  rvelcenter_pub = n.advertise<std_msgs::Float32>("/rvelcenter", 1000);
  rvelrear_pub = n.advertise<std_msgs::Float32>("/rvelrear", 1000);
  
  lWheelSubscriber = n.subscribe("/lwheel", 1000, &RobotHWInterface::lWheelCallback, this);
  rWheelSubscriber = n.subscribe("/rwheel", 1000, &RobotHWInterface::rWheelCallback, this);

  ROS_INFO_NAMED("robot_hw_interface", "RobotHWInterface Ready.");
}

void RobotHWInterface::lWheelCallback(const std_msgs::Int16 msg) {
   lWheel = msg.data;
}

void RobotHWInterface::rWheelCallback(const std_msgs::Int16 msg) {
  rWheel = msg.data;
}

void RobotHWInterface::read(ros::Duration &elapsed_time)
{
  joint_position_[0] = lWheel;
  joint_position_[1] = lWheel;
  joint_position_[2] = lWheel;
  joint_position_[3] = rWheel;
  joint_position_[4] = rWheel;
  joint_position_[5] = rWheel;
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void RobotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  std_msgs::Float32 lvelfront_vel;
  std_msgs::Float32 lvelcenter_vel;
  std_msgs::Float32 lvelrear_vel;
  std_msgs::Float32 rvelfront_vel;
  std_msgs::Float32 rvelcenter_vel;
  std_msgs::Float32 rvelrear_vel;

  lvelfront_vel.data = (float)joint_velocity_command_[0];
  lvelcenter_vel.data = (float)joint_velocity_command_[1];
  lvelrear_vel.data = (float)joint_velocity_command_[2];
  rvelfront_vel.data = (float)joint_velocity_command_[3];
  rvelcenter_vel.data = (float)joint_velocity_command_[4];
  rvelrear_vel.data = (float)joint_velocity_command_[5];

  lvelfront_pub.publish(lvelfront_vel);
  lvelcenter_pub.publish(lvelcenter_vel);
  lvelrear_pub.publish(lvelrear_vel);
  rvelfront_pub.publish(rvelfront_vel);
  rvelcenter_pub.publish(rvelcenter_vel);
  rvelrear_pub.publish(rvelrear_vel);

  // std::cout << "1: " << joint_velocity_command_[0] << " 2: " << joint_velocity_command_[1] << std::endl;

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  // for (std::size_t joint_id = 0; joint_id < 6; ++joint_id)
  // 	joint_position_[joint_id] += joint_position_command_[joint_id];
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void RobotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
