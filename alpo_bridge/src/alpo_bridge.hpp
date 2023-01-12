// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ALPO_BRIDGE_HPP_
#define ALPO_BRIDGE_HPP_

// include ROS 1
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>

// include ROS 2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <nav_msgs/msg/odometry.hpp>

// std
#include <iostream>
#include <memory>
#include <string>


using Ros1AckermannMsg = ackermann_msgs::AckermannDrive;
using Ros2AckermannMsg = ackermann_msgs::msg::AckermannDrive;
using Ros1JoyMsg = sensor_msgs::Joy;
using Ros2JoyMsg = sensor_msgs::msg::Joy;
using Ros1OdomMsg = nav_msgs::Odometry;
using Ros2OdomMsg = nav_msgs::msg::Odometry;
using Ros1JointStatesMsg = sensor_msgs::JointState;
using Ros2JointStatesMsg = sensor_msgs::msg::JointState;

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

private:
  Ros1NodePtr ros1_node_ptr_;
  Ros2NodePtr ros2_node_ptr_;

  ros::Publisher ros1_cmd_steer_pub_;
  ros::Subscriber ros1_joint_states_sub_;
  ros::Subscriber ros1_odom_sub_;
  ros::Subscriber ros1_joy_sub_;

  rclcpp::Publisher<Ros2JoyMsg>::SharedPtr ros2_joy_pub_;
  rclcpp::Publisher<Ros2OdomMsg>::SharedPtr ros2_odom_pub_;
  rclcpp::Publisher<Ros2JointStatesMsg>::SharedPtr ros2_joint_states_pub_;
  rclcpp::Subscription<Ros2AckermannMsg>::SharedPtr ros2_cmd_steer_sub_;
};


#endif  // ALPO_BRIDGE_HPP_
