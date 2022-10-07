#include "alpo_bridge.hpp"

const std::string alpo_joy_topic = "/joy";
const std::string alpo_odom_topic = "/alpo_driver/ackermann_controller/odom";
const std::string alpo_cmd_steer_topic = "/auto/cmd_steer";
const std::string alpo_joint_states_topic = "/alpo_driver/joint_states";

const std::string bridge_joy_topic = "/alpo_bridge/joy";
const std::string bridge_odom_topic = "/alpo_bridge/vehicle_controller/odom";
const std::string bridge_cmd_steer_topic = "/alpo_bridge/vehicle_controller/cmd_steer";
const std::string bridge_joint_states_topic = "/alpo_bridge/vehicle_controller/joint_states";

const rclcpp::QoS data_qos= rclcpp::SensorDataQoS().reliable();
const rclcpp::QoS cmd_qos= rclcpp::QoS(rclcpp::KeepLast(1)).
    best_effort().durability_volatile();

//-----------------------------------------------------------------------------
AlpoBridge::AlpoBridge(Ros1NodePtr ros1_node_ptr,
                       Ros2NodePtr ros2_node_ptr):
  ros1_node_ptr_(ros1_node_ptr),
  ros2_node_ptr_(ros2_node_ptr)
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

  std::copy( ros1_msg->pose.covariance.begin(),
             ros1_msg->pose.covariance.end(),
             ros2_msg.pose.covariance.begin());

  ros2_msg.twist.twist.linear.x = ros1_msg->twist.twist.linear.x;
  ros2_msg.twist.twist.linear.y = ros1_msg->twist.twist.linear.y;
  ros2_msg.twist.twist.linear.z = ros1_msg->twist.twist.linear.z;
  ros2_msg.twist.twist.angular.x = ros1_msg->twist.twist.angular.x;
  ros2_msg.twist.twist.angular.y = ros1_msg->twist.twist.angular.y;
  ros2_msg.twist.twist.angular.z = ros1_msg->twist.twist.angular.z;

  std::copy( ros1_msg->twist.covariance.begin(),
             ros1_msg->twist.covariance.end(),
             ros2_msg.twist.covariance.begin());

  ros2_odom_pub_->publish(ros2_msg);
}

//-----------------------------------------------------------------------------
void AlpoBridge::ros1_joint_states_callback_(const Ros1JointStatesMsg::ConstPtr & ros1_msg)
{
  Ros2JointStatesMsg ros2_msg;
  ros2_msg.name = ros1_msg->name;
  ros2_msg.position = ros1_msg->position;
  ros2_msg.velocity = ros1_msg->velocity;
  ros2_msg.effort = ros1_msg->effort;
  ros2_joint_states_pub_->publish(ros2_msg);
}

//-----------------------------------------------------------------------------
void AlpoBridge::start()
{
  init_ros1_publisher_();
  init_ros2_publishers_();
  init_ros1_subscriptions_();
  init_ros2_subcription_();
}

//-----------------------------------------------------------------------------
void AlpoBridge::init_ros1_publisher_()
{
  ros1_cmd_steer_pub_ =  ros1_node_ptr_->
      advertise<Ros1AckermannMsg>(alpo_cmd_steer_topic, 1);
}

//-----------------------------------------------------------------------------
void AlpoBridge::init_ros2_publishers_()
{
  ros2_joy_pub_ = ros2_node_ptr_->
      create_publisher<Ros2JoyMsg>(bridge_joy_topic, data_qos);
  ros2_odom_pub_ = ros2_node_ptr_->
      create_publisher<Ros2OdomMsg>(bridge_odom_topic, data_qos);
  ros2_joint_states_pub_ = ros2_node_ptr_->
      create_publisher<Ros2JointStatesMsg>(bridge_joint_states_topic, data_qos);
}

//-----------------------------------------------------------------------------
void AlpoBridge::init_ros1_subscriptions_()
{
    ros1_joint_states_sub_ = ros1_node_ptr_->subscribe(
          alpo_joint_states_topic, 10, &AlpoBridge::ros1_joint_states_callback_,this);
    ros1_odom_sub_ = ros1_node_ptr_->subscribe(
          alpo_odom_topic, 10, &AlpoBridge::ros1_odometry_callback_,this);
    ros1_joy_sub_ = ros1_node_ptr_->subscribe(
          alpo_joy_topic, 10, &AlpoBridge::ros1_joy_callback_,this);
}

//-----------------------------------------------------------------------------
void AlpoBridge::init_ros2_subcription_()
{
    rclcpp::SubscriptionOptions options;
    options.ignore_local_publications = true;

    auto callback = std::bind(&AlpoBridge::ros2_cmd_steer_callback_,
                              this,std::placeholders::_1);

    ros2_cmd_steer_sub_ = ros2_node_ptr_->create_subscription<Ros2AckermannMsg>(
          bridge_cmd_steer_topic, cmd_qos, callback,options);
}
