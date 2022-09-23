#ifndef ALPO_HARDWARE_HPP_
#define ALPO_HARDWARE_HPP_

//romea
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"

//ros2
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

//std
#include <atomic>
#include <fstream>

namespace romea
{

template <typename HardwareInterface>
class AlpoHardware : public HardwareSystemInterface<HardwareInterface>
{  

public:

  RCLCPP_SHARED_PTR_DEFINITIONS(AlpoHardware);

  AlpoHardware();

  virtual hardware_interface::return_type read() override;

  virtual hardware_interface::return_type write() override;

private:

  virtual hardware_interface::return_type connect_() override;

  virtual hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
      const hardware_interface::HardwareInfo & hardware_info) override ;

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

}

#endif