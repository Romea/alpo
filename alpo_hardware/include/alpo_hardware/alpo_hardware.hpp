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


#ifndef ALPO_HARDWARE__ALPO_HARDWARE_HPP_
#define ALPO_HARDWARE__ALPO_HARDWARE_HPP_

// std
#include <atomic>
#include <fstream>

// romea
#include "romea_common_utils/ros_versions.hpp"
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"

// ros2
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"


namespace romea
{
namespace ros2
{

template<typename HardwareInterface>
class AlpoHardware : public HardwareSystemInterface<HardwareInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AlpoHardware);

  AlpoHardware();

#if ROS_DISTRO == ROS_GALACTIC
  hardware_interface::return_type read()override;

  hardware_interface::return_type write()override;
#else
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;
#endif

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

private:
  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info) override;

  void joint_states_callback_(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

  void send_command_();

  void send_null_command_();

  void get_hardware_command_();

  void set_hardware_state_();


#ifndef NDEBUG
  void open_log_file_();
  void write_log_header_();
  void write_log_data_();
#endif

private:
  float front_wheel_radius_;
  float rear_wheel_radius_;
  double wheelbase_;
  double front_track_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_steer_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  std::atomic<float> front_left_wheel_steering_angle_measure_;
  std::atomic<float> front_right_wheel_steering_angle_measure_;
  std::atomic<float> front_left_wheel_angular_speed_measure_;
  std::atomic<float> front_right_wheel_angular_speed_measure_;
  std::atomic<float> rear_left_wheel_angular_speed_measure_;
  std::atomic<float> rear_right_wheel_angular_speed_measure_;

  float front_left_wheel_steering_angle_command_;
  float front_right_wheel_steering_angle_command_;
  float front_left_wheel_angular_speed_command_;
  float front_right_wheel_angular_speed_command_;
  float rear_left_wheel_angular_speed_command_;
  float rear_right_wheel_angular_speed_command_;

#ifndef NDEBUG
  std::fstream debug_file_;
#endif
};

using AlpoHardware2FWS4WD = AlpoHardware<HardwareInterface2FWS4WD>;
using AlpoHardware2FWS2RWD = AlpoHardware<HardwareInterface2FWS2RWD>;

}  // namespace ros2
}  // namespace romea

#endif  // ALPO_HARDWARE__ALPO_HARDWARE_HPP_
