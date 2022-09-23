#include "alpo_hardware/alpo_hardware.hpp"

#include <romea_common_utils/qos.hpp>
#include <romea_mobile_base_hardware/hardware_info.hpp>
#include <romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp>


namespace  {

size_t joint_id(const std::vector<std::string> joint_state_names,
                const std::string & joint_name)
{
  auto it = std::find(joint_state_names.cbegin(),
                      joint_state_names.cend(),
                      joint_name);

  if(it==joint_state_names.end())
  {
    throw std::runtime_error("Cannot find info of "+ joint_name + " in joint_states msg");
  }

  return std::distance(joint_state_names.cbegin(),it);
}

const double & position(const sensor_msgs::msg::JointState & joint_states,const std::string & joint_name)
{
  return joint_states.position[joint_id(joint_states.name,joint_name)];
}

const double & velocity(const sensor_msgs::msg::JointState & joint_states,const std::string & joint_name)
{
  return joint_states.velocity[joint_id(joint_states.name,joint_name)];
}

}

namespace romea {

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
AlpoHardware<HardwareInterface>::AlpoHardware():
  HardwareSystemInterface<HardwareInterface>(),
  front_wheel_radius_(0),
  rear_wheel_radius_(0),
//  front_left_wheel_steering_angle_measure_(0),
//  front_right_wheel_steering_angle_measure_(0),
//  front_left_wheel_angular_speed_measure_(0),
//  front_right_wheel_angular_speed_measure_(0),
//  rear_left_wheel_angular_speed_measure_(0),
//  rear_right_wheel_angular_speed_measure_(0),
  front_left_wheel_steering_angle_measure_(std::numeric_limits<double>::quiet_NaN()),
  front_right_wheel_steering_angle_measure_(std::numeric_limits<double>::quiet_NaN()),
  front_left_wheel_angular_speed_measure_(std::numeric_limits<double>::quiet_NaN()),
  front_right_wheel_angular_speed_measure_(std::numeric_limits<double>::quiet_NaN()),
  rear_left_wheel_angular_speed_measure_(std::numeric_limits<double>::quiet_NaN()),
  rear_right_wheel_angular_speed_measure_(std::numeric_limits<double>::quiet_NaN()),
  front_left_wheel_steering_angle_command_(0),
  front_right_wheel_steering_angle_command_(0),
  front_left_wheel_angular_speed_command_(0),
  front_right_wheel_angular_speed_command_(0),
  rear_left_wheel_angular_speed_command_(0),
  rear_right_wheel_angular_speed_command_(0)
{
#ifndef NDEBUG
  open_log_file_();
  write_log_header_();
#endif


  node_ = rclcpp::Node::make_shared("mobile_base_controller_bridge");
  auto callback = std::bind(&AlpoHardware<HardwareInterface>::joint_states_callback_,this,std::placeholders::_1);
  cmd_steer_pub_ = node_->create_publisher<ackermann_msgs::msg::AckermannDrive>("/alpo_bridge/vehicle_controller/cmd_steer",sensor_data_qos());
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/alpo_bridge/vehicle_controller/joint_states",best_effort(1),callback);

}



//-----------------------------------------------------------------------------
template<typename HardwareInterface>
hardware_interface::return_type AlpoHardware<HardwareInterface>::connect_()
{
  RCLCPP_ERROR(rclcpp::get_logger("AlpoHardware"), "Init communication with robot");

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
hardware_interface::return_type AlpoHardware<HardwareInterface>::disconnect_()
{
  RCLCPP_ERROR(rclcpp::get_logger("Adap2eHardware"), "Close communication with robot");

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
hardware_interface::return_type AlpoHardware<HardwareInterface>::load_info_(
    const hardware_interface::HardwareInfo & hardware_info)
{

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlpoHardware"),"load_info");

  try {
    wheelbase_ = get_parameter<double>(hardware_info,"wheelbase");
    front_track_= get_parameter<double>(hardware_info,"front_track");
    front_wheel_radius_=get_parameter<float>(hardware_info,"front_wheel_radius");
    rear_wheel_radius_=get_parameter<float>(hardware_info,"rear_wheel_radius");
    return hardware_interface::return_type::OK;
  }
  catch (std::runtime_error &e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("AlpoHardware"),e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
void AlpoHardware<HardwareInterface>::send_command_()
{

  ackermann_msgs::msg::AckermannDrive cmd;

  cmd.speed = 0.5*rear_wheel_radius_*
      (rear_left_wheel_angular_speed_command_+
       rear_right_wheel_angular_speed_command_);

  cmd.steering_angle = romea::TwoWheelSteeringKinematic::
      computeSteeringAngle(front_left_wheel_steering_angle_command_,
                           front_right_wheel_steering_angle_command_,
                           wheelbase_,front_track_);

  cmd_steer_pub_->publish(cmd);
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
void AlpoHardware<HardwareInterface>::send_null_command_()
{
  cmd_steer_pub_->publish(ackermann_msgs::msg::AckermannDrive());
}


//-----------------------------------------------------------------------------
template<typename HardwareInteface>
hardware_interface::return_type AlpoHardware<HardwareInteface>::read()
{
//    RCLCPP_INFO(rclcpp::get_logger("AlpoHardware"), "Read data from robot");
    rclcpp::spin_some(node_);

    try {
    set_hardware_state_();
#ifndef NDEBUG
    write_log_data_();
#endif
    return hardware_interface::return_type::OK;
  }
  catch (std::runtime_error & e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("AlpoHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}


//-----------------------------------------------------------------------------
template<typename HardwareInteface>
hardware_interface::return_type AlpoHardware<HardwareInteface>::write()
{
//  RCLCPP_INFO(rclcpp::get_logger("AlpoHardware"), "Send command to robot");

  send_command_();

  return hardware_interface::return_type::OK;

}

//-----------------------------------------------------------------------------
template<>
void AlpoHardware<HardwareInterface2FWS4WD>::joint_states_callback_(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  front_left_wheel_steering_angle_measure_=
      position(*msg,"front_left_wheel_steering_joint");
  front_right_wheel_steering_angle_measure_=
        position(*msg,"front_right_wheel_steering_joint");
  front_left_wheel_angular_speed_measure_=
      velocity(*msg,"front_left_wheel_spinning_joint");
  front_right_wheel_angular_speed_measure_=
      velocity(*msg,"front_right_wheel_spinning_joint");
  rear_left_wheel_angular_speed_measure_=
      velocity(*msg,"rear_left_wheel_spinning_joint");
  rear_right_wheel_angular_speed_measure_=
      velocity(*msg,"rear_right_wheel_spinning_joint");
}

//-----------------------------------------------------------------------------
template<>
void AlpoHardware<HardwareInterface2FWS2RWD>::joint_states_callback_(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  front_left_wheel_steering_angle_measure_=
      position(*msg,"front_left_wheel_steering_joint");
  front_right_wheel_steering_angle_measure_=
      position(*msg,"front_right_wheel_steering_joint");
  rear_left_wheel_angular_speed_measure_=
      velocity(*msg,"rear_left_wheel_spinning_joint");
  rear_right_wheel_angular_speed_measure_=
      velocity(*msg,"rear_right_wheel_spinning_joint");
}

//-----------------------------------------------------------------------------
template<>
void AlpoHardware<HardwareInterface2FWS2RWD>::set_hardware_state_()
{
  HardwareState2FWS2RWD state;
  state.frontLeftWheelSteeringAngle = front_left_wheel_steering_angle_measure_;
  state.frontRightWheelSteeringAngle = front_right_wheel_steering_angle_measure_;
  state.rearLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.rearRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  this->hardware_interface_->set_state(state);
}

//-----------------------------------------------------------------------------
template<>
void AlpoHardware<HardwareInterface2FWS4WD>::set_hardware_state_()
{
  HardwareState2FWS4WD state;
  state.frontLeftWheelSteeringAngle = front_left_wheel_steering_angle_measure_;
  state.frontRightWheelSteeringAngle = front_right_wheel_steering_angle_measure_;
  state.frontLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.frontRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  state.rearLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.rearRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  this->hardware_interface_->set_state(state);
}

//-----------------------------------------------------------------------------
template<>
void AlpoHardware<HardwareInterface2FWS2RWD>::get_hardware_command_()
{
  HardwareCommand2FWS2RWD command = hardware_interface_->get_command();

  front_left_wheel_steering_angle_command_ = command.frontLeftWheelSteeringAngle;
  front_right_wheel_steering_angle_command_ = command.frontRightWheelSteeringAngle;
  rear_left_wheel_angular_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_angular_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

//-----------------------------------------------------------------------------
template<>
void AlpoHardware<HardwareInterface2FWS4WD>::get_hardware_command_()
{
  HardwareCommand2FWS4WD command = hardware_interface_->get_command();
  front_left_wheel_steering_angle_command_ = command.frontLeftWheelSteeringAngle;
  front_right_wheel_steering_angle_command_ = command.frontRightWheelSteeringAngle;
  front_left_wheel_angular_speed_command_ = command.frontLeftWheelSpinningSetPoint;
  front_right_wheel_angular_speed_command_ = command.frontRightWheelSpinningSetPoint;
  rear_left_wheel_angular_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_angular_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

#ifndef NDEBUG
//-----------------------------------------------------------------------------
template<typename HardwareInterface>
void AlpoHardware<HardwareInterface>::open_log_file_()
{
  debug_file_.open(std::string("alpo.dat").c_str(),
                   std::fstream::in|std::fstream::out|std::fstream::trunc);
}
//-----------------------------------------------------------------------------
template<typename HardwareInterface>
void AlpoHardware<HardwareInterface>::write_log_header_()
{
  if(debug_file_.is_open())
  {
    debug_file_ <<"# time, ";
    debug_file_ <<" FLS, "<<" FRS, ";
    debug_file_ <<" RLS, "<<" RRS, ";
    debug_file_ <<" FLA, "<<" FRA, ";
    debug_file_ <<" FLS_cmd, "<<" FRS_cmd, ";
    debug_file_ <<" RLS_cmd, "<<" RRS_cmd, ";
    debug_file_ <<" FLA_cmd, "<<" FRA_cmd, "<<"\n";
  }
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
void AlpoHardware<HardwareInterface>::write_log_data_()
{
  if(debug_file_.is_open())
  {
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    debug_file_ << std::setprecision(10);
    debug_file_ << now_ns.time_since_epoch().count()<<" ";
    debug_file_ <<front_left_wheel_angular_speed_measure_*front_wheel_radius_<<" ";
    debug_file_ << front_right_wheel_angular_speed_measure_*front_wheel_radius_<<" ";
    debug_file_ <<rear_left_wheel_angular_speed_measure_*rear_wheel_radius_<<" ";
    debug_file_ << rear_right_wheel_angular_speed_measure_*rear_wheel_radius_<<" ";
    debug_file_ <<front_left_wheel_steering_angle_measure_<<" ";
    debug_file_ << front_right_wheel_steering_angle_measure_<<" ";
    debug_file_ <<front_left_wheel_angular_speed_command_*front_wheel_radius_<<" ";
    debug_file_ << front_right_wheel_angular_speed_command_*front_wheel_radius_<<" ";
    debug_file_ <<rear_left_wheel_angular_speed_command_*rear_wheel_radius_<<" ";
    debug_file_ << rear_right_wheel_angular_speed_command_*rear_wheel_radius_<<" ";
    debug_file_ <<front_left_wheel_steering_angle_command_<<" ";
    debug_file_ << front_right_wheel_steering_angle_command_<<" ";
  }
}
#endif

template class AlpoHardware<HardwareInterface2FWS4WD>;
template class AlpoHardware<HardwareInterface2FWS2RWD>;

}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::AlpoHardware2FWS4WD, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(romea::AlpoHardware2FWS2RWD, hardware_interface::SystemInterface)