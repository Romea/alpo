# 1 Overview #

As the low-level interface of alpos robots runs under ROS 1, in order to communicate with them a bridge between ROS1 and ROS2 is provided in this package. This bridge is used to broadcast messages coming from the embedded joystick and robot's controller (wheel joint states and odometry information) and to send command to the robot.

# 2 Node #

## 2.1 ROS1 ##

### 2.1.1 Subscribed Topics ###

- /alpo_driver/ackermann_controller/odom (nav_msgs::Odometry)

  This topic is pusblished by alpo controller and provides a lot informations about robot kinematic and dead reckoning.

- /alpo_driver/joint_states (sensors_msgs::JointState)

  This topic is also published by alpo controller an provides informations about wheel joints states

- /joy (sensor_msgs::msg::Joy)

  This topic is published by the HJ60 joystick which is used to control implements and to switch between automatic and manual modes 

### 2.1.2 Published Topics ###

- /auto/cmd_steer (ackermann_msgs::AckermannDrive)

  This topic is send to alpo controller and contains ackermann command provided by high level algorithms

## 2.2 ROS2 ##

### 2.2.1 Subscribed Topics ###

- ~vehicle_controller/cmd_steer (ackermann_msgs::msg::AckermannDrive)

  This topic contains ackermann command send by high level algorithms to the robot  

### 2.2.2 Published Topics ###

- ~vehicle_controller/odom (nav_msgs::msg::Odometry)

  This topic is broadcasted by the bridge and provides odometry message computed by alpo controller 

- ~vehicle_controller/joint_states (sensor_msgs::msg::JointState)

  This topic is broadcasted by the bridge and provides joint states computed by alpo controller 

- ~joy (sensor_msgs::msg::Joy)

  This topic is broadcasted by the bridge and provides joy messages send by HJ60 embedded joystick 


# 3 Script #

In order to simplify the launch of the bridge, a script has been provided. It is used to source ROS1 noetic distribution and then to launch the bridge node. It can be used as below:

    ros2 run alpo_bridge alpo_bridge_node
