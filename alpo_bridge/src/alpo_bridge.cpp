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

#include "alpo_bridge.hpp"

const char alpo_joy_topic[] = "/joy";
const char alpo_odom_topic[] = "/alpo_driver/ackermann_controller/odom";
const char alpo_cmd_steer_topic[] = "/auto/cmd_steer";
const char alpo_joint_states_topic[] = "/alpo_driver/joint_states";
const char alpo_battery_state_topic[] = "/power_supply/battery_status";

const char alpo_impl_front_cmd_topic[] = "/cylinder/front/interface/command";
const char alpo_impl_rear_cmd_topic[] = "/cylinder/rear/interface/command";

const char bridge_joy_topic[] = "~/joy";
const char bridge_odom_topic[] = "~/vehicle_controller/odom";
const char bridge_cmd_steer_topic[] = "~/vehicle_controller/cmd_steer";
const char bridge_joint_states_topic[] = "~/vehicle_controller/joint_states";
const char bridge_battery_state_topic[] = "~/power_supply/battery_status";
const char bridge_impl_front_cmd_topic[] = "implement/front/command";
const char bridge_impl_rear_cmd_topic[] = "implement/rear/command";

const rclcpp::QoS data_qos = rclcpp::SensorDataQoS().reliable();
const rclcpp::QoS cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

//-----------------------------------------------------------------------------
AlpoBridge::AlpoBridge(Ros1NodePtr ros1_node_ptr, Ros2NodePtr ros2_node_ptr)
: ros1_node_ptr_(ros1_node_ptr), ros2_node_ptr_(ros2_node_ptr)
{
}

//-----------------------------------------------------------------------------
void AlpoBridge::ros2_cmd_steer_callback_(const Ros2AckermannMsg::SharedPtr ros2_msg)
{
  Ros1AckermannMsg ros1_msg;
  ros1_msg.jerk = ros2_msg->jerk;
  ros1_msg.speed = ros2_msg->speed;
  ros1_msg.acceleration = ros2_msg->acceleration;
  ros1_msg.steering_angle = ros2_msg->steering_angle;
  ros1_msg.steering_angle_velocity = ros2_msg->steering_angle_velocity;
  ros1_cmd_steer_pub_.publish(ros1_msg);
}

//-----------------------------------------------------------------------------
void AlpoBridge::ros2_impl_cmd_callback_(
  const Ros2ImplCmdMsg::SharedPtr ros2_msg, const char * side)
{
  Ros1ImplCmdMsg ros1_msg;
  if (ros2_msg->command == Ros2ImplCmdMsg::NONE) {
    ros1_msg.command = Ros1ImplCmdMsg::STOP;
  } else if (ros2_msg->command == Ros2ImplCmdMsg::NONE) {
    ros1_msg.command = Ros1ImplCmdMsg::STOP;
  } else if (ros2_msg->command == Ros2ImplCmdMsg::GO_DOWN) {
    ros1_msg.command = Ros1ImplCmdMsg::GO_DOWN;
  } else if (ros2_msg->command == Ros2ImplCmdMsg::GO_UP) {
    ros1_msg.command = Ros1ImplCmdMsg::GO_UP;
  } else if (ros2_msg->command == Ros2ImplCmdMsg::GO_TO_ANCHOR_LOW) {
    ros1_msg.command = Ros1ImplCmdMsg::GO_TO_ANCHOR_LOW;
  } else if (ros2_msg->command == Ros2ImplCmdMsg::GO_TO_ANCHOR_HIGH) {
    ros1_msg.command = Ros1ImplCmdMsg::GO_TO_ANCHOR_HIGH;
  }

  if (!strcmp(side, "front"))
    ros1_impl_front_cmd_pub_.publish(ros1_msg);
  else
    ros1_impl_rear_cmd_pub_.publish(ros1_msg);
}

//-----------------------------------------------------------------------------
void AlpoBridge::ros1_joy_callback_(const Ros1JoyMsg::ConstPtr ros1_msg)
{
  Ros2JoyMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  ros2_msg.header.frame_id = ros1_msg->header.frame_id;
  ros2_msg.buttons = ros1_msg->buttons;
  ros2_msg.axes = ros1_msg->axes;
  ros2_joy_pub_->publish(ros2_msg);
}

//-----------------------------------------------------------------------------
void AlpoBridge::ros1_odometry_callback_(const Ros1OdomMsg::ConstPtr & ros1_msg)
{
  Ros2OdomMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  ros2_msg.header.frame_id = ros1_msg->header.frame_id;
  ros2_msg.child_frame_id = ros1_msg->child_frame_id;

  ros2_msg.pose.pose.position.x = ros1_msg->pose.pose.position.x;
  ros2_msg.pose.pose.position.y = ros1_msg->pose.pose.position.y;
  ros2_msg.pose.pose.position.z = ros1_msg->pose.pose.position.z;
  ros2_msg.pose.pose.orientation.x = ros1_msg->pose.pose.orientation.x;
  ros2_msg.pose.pose.orientation.y = ros1_msg->pose.pose.orientation.y;
  ros2_msg.pose.pose.orientation.z = ros1_msg->pose.pose.orientation.z;
  ros2_msg.pose.pose.orientation.w = ros1_msg->pose.pose.orientation.w;

  std::copy(
    ros1_msg->pose.covariance.begin(),
    ros1_msg->pose.covariance.end(),
    ros2_msg.pose.covariance.begin());

  ros2_msg.twist.twist.linear.x = ros1_msg->twist.twist.linear.x;
  ros2_msg.twist.twist.linear.y = ros1_msg->twist.twist.linear.y;
  ros2_msg.twist.twist.linear.z = ros1_msg->twist.twist.linear.z;
  ros2_msg.twist.twist.angular.x = ros1_msg->twist.twist.angular.x;
  ros2_msg.twist.twist.angular.y = ros1_msg->twist.twist.angular.y;
  ros2_msg.twist.twist.angular.z = ros1_msg->twist.twist.angular.z;

  std::copy(
    ros1_msg->twist.covariance.begin(),
    ros1_msg->twist.covariance.end(),
    ros2_msg.twist.covariance.begin());

  ros2_odom_pub_->publish(ros2_msg);
}

//-----------------------------------------------------------------------------
void AlpoBridge::ros1_joint_states_callback_(const Ros1JointStatesMsg::ConstPtr & ros1_msg)
{
  Ros2JointStatesMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  ros2_msg.header.frame_id = ros1_msg->header.frame_id;
  ros2_msg.name = ros1_msg->name;
  ros2_msg.position = ros1_msg->position;
  ros2_msg.velocity = ros1_msg->velocity;
  ros2_msg.effort = ros1_msg->effort;
  ros2_joint_states_pub_->publish(ros2_msg);
}

//-----------------------------------------------------------------------------
void AlpoBridge::ros1_battery_state_callback_(const Ros1BatteryStateMsg::ConstPtr & ros1_msg)
{
  Ros2BatteryStateMsg ros2_msg;
  ros2_msg.header.stamp = ros2_node_ptr_->get_clock()->now();
  ros2_msg.header.frame_id = ros1_msg->header.frame_id;
  ros2_msg.voltage = ros1_msg->voltage;
  ros2_msg.temperature = ros1_msg->temperature;
  ros2_msg.current = ros1_msg->current;
  ros2_msg.charge = ros1_msg->charge;
  ros2_msg.capacity = ros1_msg->capacity;
  ros2_msg.design_capacity = ros1_msg->design_capacity;
  ros2_msg.percentage = ros1_msg->percentage;
  ros2_msg.power_supply_status = ros1_msg->power_supply_status;
  ros2_msg.power_supply_health = ros1_msg->power_supply_health;
  ros2_msg.power_supply_technology = ros1_msg->power_supply_technology;
  ros2_msg.present = ros1_msg->present;
  ros2_msg.cell_voltage = ros1_msg->cell_voltage;
  ros2_msg.cell_temperature = ros1_msg->cell_temperature;
  ros2_msg.location = ros1_msg->location;
  ros2_msg.serial_number = ros1_msg->serial_number;
  ros2_battery_state_pub_->publish(ros2_msg);
}
//-----------------------------------------------------------------------------
void AlpoBridge::start()
{
  init_ros1_publisher_();
  init_ros2_publishers_();
  init_ros1_subscriptions_();
  init_ros2_subcription_();

  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create sub 2 -> 1: " << bridge_cmd_steer_topic << " -> " << alpo_cmd_steer_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create sub 2 -> 1: " << bridge_impl_front_cmd_topic << " -> " << alpo_impl_front_cmd_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create sub 2 -> 1: " << bridge_impl_rear_cmd_topic << " -> " << alpo_impl_rear_cmd_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_joy_topic << " -> " << alpo_joy_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_odom_topic << " -> " << alpo_odom_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_joint_states_topic << " -> " << alpo_joint_states_topic);
  RCLCPP_INFO_STREAM(
    ros2_node_ptr_->get_logger(),
    "Create pub 2 <- 1: " << bridge_battery_state_topic << " -> " << alpo_battery_state_topic);
}

//-----------------------------------------------------------------------------
void AlpoBridge::init_ros1_publisher_()
{
  ros1_cmd_steer_pub_ = ros1_node_ptr_->advertise<Ros1AckermannMsg>(alpo_cmd_steer_topic, 1);
  ros1_impl_front_cmd_pub_ =
    ros1_node_ptr_->advertise<Ros1ImplCmdMsg>(alpo_impl_front_cmd_topic, 1);
  ros1_impl_rear_cmd_pub_ = ros1_node_ptr_->advertise<Ros1ImplCmdMsg>(alpo_impl_rear_cmd_topic, 1);
}

//-----------------------------------------------------------------------------
void AlpoBridge::init_ros2_publishers_()
{
  ros2_joy_pub_ = ros2_node_ptr_->create_publisher<Ros2JoyMsg>(bridge_joy_topic, data_qos);
  ros2_odom_pub_ = ros2_node_ptr_->create_publisher<Ros2OdomMsg>(bridge_odom_topic, data_qos);
  ros2_joint_states_pub_ =
    ros2_node_ptr_->create_publisher<Ros2JointStatesMsg>(bridge_joint_states_topic, data_qos);
  ros2_battery_state_pub_ =
    ros2_node_ptr_->create_publisher<Ros2BatteryStateMsg>(bridge_battery_state_topic, data_qos);
}

//-----------------------------------------------------------------------------
void AlpoBridge::init_ros1_subscriptions_()
{
  ros1_battery_state_sub_ = ros1_node_ptr_->subscribe(
    alpo_battery_state_topic, 10, &AlpoBridge::ros1_battery_state_callback_, this);
  ros1_joint_states_sub_ = ros1_node_ptr_->subscribe(
    alpo_joint_states_topic, 10, &AlpoBridge::ros1_joint_states_callback_, this);
  ros1_odom_sub_ =
    ros1_node_ptr_->subscribe(alpo_odom_topic, 10, &AlpoBridge::ros1_odometry_callback_, this);
  ros1_joy_sub_ =
    ros1_node_ptr_->subscribe(alpo_joy_topic, 10, &AlpoBridge::ros1_joy_callback_, this);
}

//-----------------------------------------------------------------------------
void AlpoBridge::init_ros2_subcription_()
{
  rclcpp::SubscriptionOptions options;
  options.ignore_local_publications = true;

  auto callback = std::bind(&AlpoBridge::ros2_cmd_steer_callback_, this, std::placeholders::_1);

  ros2_cmd_steer_sub_ = ros2_node_ptr_->create_subscription<Ros2AckermannMsg>(
    bridge_cmd_steer_topic, cmd_qos, callback, options);

  auto impl_front_cmd_callback = [this](const Ros2ImplCmdMsg::SharedPtr msg) -> void {
    ros2_impl_cmd_callback_(msg, "front");
  };
  ros2_impl_front_cmd_sub_ = ros2_node_ptr_->create_subscription<Ros2ImplCmdMsg>(
    bridge_impl_front_cmd_topic, cmd_qos, impl_front_cmd_callback, options);

  auto impl_rear_cmd_callback = [this](const Ros2ImplCmdMsg::SharedPtr msg) -> void {
    ros2_impl_cmd_callback_(msg, "rear");
  };
  ros2_impl_rear_cmd_sub_ = ros2_node_ptr_->create_subscription<Ros2ImplCmdMsg>(
    bridge_impl_rear_cmd_topic, cmd_qos, impl_rear_cmd_callback, options);
}
