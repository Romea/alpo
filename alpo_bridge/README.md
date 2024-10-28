# alpo_bridge #

## 1 Overview ##

The **Alpo Bridge** package provides a communication bridge between ROS1 and ROS2 to interface with **Alpo** robots, which run on a low-level ROS1 interface. This bridge allows the transmission of data from the robot’s embedded joystick and controller (including wheel joint states and odometry information) and enables command transmission to the robot.

## 2 Bridge node description

### 2.1 ROS1 side ###

#### 2.1.1 Subscribed Topics ####

- **/alpo_driver/ackermann_controller/odom** (nav_msgs::Odometry)

  This topic is pusblished by alpo controller and provides a lot informations about robot kinematic and dead reckoning.

- **/alpo_driver/joint_states** (sensors_msgs::JointState)

  This topic is also published by alpo controller an provides informations about wheel joints states

- **/joy** (sensor_msgs::msg::Joy)

  This topic is published by the HJ60 joystick which is used to control implements and to switch between automatic and manual modes 

#### 2.1.2 Published Topics ####

- **/auto/cmd_steer** (ackermann_msgs::AckermannDrive)

  This topic is send to alpo controller and contains ackermann command provided by high level algorithms

### 2.2 ROS2 side ###

#### 2.2.1 Subscribed Topics

- **~vehicle_controller/cmd_steer** (ackermann_msgs::msg::AckermannDrive)

  This topic contains ackermann command send by high level algorithms to the robot  

#### 2.2.2 Published Topics ####

- **~vehicle_controller/odom** (nav_msgs::msg::Odometry)

  This topic is broadcasted by the bridge and provides odometry message computed by Alpo controller 

- **~vehicle_controller/joint_states** (sensor_msgs::msg::JointState)

  This topic is broadcasted by the bridge and provides joint states computed by Alpo controller 

- **~joy** (sensor_msgs::msg::Joy)

  This topic is broadcasted by the bridge and provides joy messages send by HJ60 embedded joystick 


## 3 Script ##

A script is provided to help to launch the bridge. This script automatically sources the ROS1 Noetic environment and launches the bridge node. You can run the script as follows:

    ros2 run alpo_bridge alpo_bridge_node
