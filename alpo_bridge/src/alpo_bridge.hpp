// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ALPO_BRIDGE_HPP_
#define ALPO_BRIDGE_HPP_

// std
#include <iostream>
#include <memory>
#include <string>

// include ROS 1
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "nav_msgs/Odometry.h"
#include "linak_a36_msgs/CylinderCommand.h"

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "romea_implement_msgs/msg/command.hpp"
#include "romea_implement_msgs/msg/state.hpp"


using Ros1AckermannMsg = ackermann_msgs::AckermannDrive;
using Ros2AckermannMsg = ackermann_msgs::msg::AckermannDrive;
using Ros1JoyMsg = sensor_msgs::Joy;
using Ros2JoyMsg = sensor_msgs::msg::Joy;
using Ros1OdomMsg = nav_msgs::Odometry;
using Ros2OdomMsg = nav_msgs::msg::Odometry;
using Ros1JointStatesMsg = sensor_msgs::JointState;
using Ros2JointStatesMsg = sensor_msgs::msg::JointState;
using Ros1ImplCmdMsg = linak_a36_msgs::CylinderCommand;
using Ros2ImplCmdMsg = romea_implement_msgs::msg::Command;

using Ros1NodePtr = std::shared_ptr<ros::NodeHandle>;
using Ros2NodePtr = std::shared_ptr<rclcpp::Node>;

class AlpoBridge
{
public:
  AlpoBridge(
    Ros1NodePtr ros1_node_ptr,
    Ros2NodePtr ros2_node_ptr);

  void start();

private:
  void init_ros1_publisher_();
  void init_ros2_publishers_();
  void init_ros1_subscriptions_();
  void init_ros2_subcription_();

  void ros1_joy_callback_(const Ros1JoyMsg::ConstPtr ros1_msg);
  void ros1_odometry_callback_(const Ros1OdomMsg::ConstPtr & ros1_msg);
  void ros1_joint_states_callback_(const Ros1JointStatesMsg::ConstPtr & ros1_msg);

  void ros2_cmd_steer_callback_(const Ros2AckermannMsg::SharedPtr ros2_msg);
  void ros2_impl_cmd_callback_(const Ros2ImplCmdMsg::SharedPtr ros2_msg, const char * side);

private:
  Ros1NodePtr ros1_node_ptr_;
  Ros2NodePtr ros2_node_ptr_;

  ros::Publisher ros1_cmd_steer_pub_;
  ros::Subscriber ros1_joint_states_sub_;
  ros::Subscriber ros1_odom_sub_;
  ros::Subscriber ros1_joy_sub_;

  // Implement topics
  ros::Publisher ros1_impl_front_cmd_pub_;
  ros::Publisher ros1_impl_rear_cmd_pub_;

  rclcpp::Publisher<Ros2JoyMsg>::SharedPtr ros2_joy_pub_;
  rclcpp::Publisher<Ros2OdomMsg>::SharedPtr ros2_odom_pub_;
  rclcpp::Publisher<Ros2JointStatesMsg>::SharedPtr ros2_joint_states_pub_;
  rclcpp::Subscription<Ros2AckermannMsg>::SharedPtr ros2_cmd_steer_sub_;

  // Implement topics
  rclcpp::Subscription<Ros2ImplCmdMsg>::SharedPtr ros2_impl_front_cmd_sub_;
  rclcpp::Subscription<Ros2ImplCmdMsg>::SharedPtr ros2_impl_rear_cmd_sub_;
};

#endif  // ALPO_BRIDGE_HPP_
