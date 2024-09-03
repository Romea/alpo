# 1 Overview #



# 2 Bridge package

Embedded PC used to achieve low control of ALPO's robot is under ROS1. In order to control ALPO robot under ROS2  a bridge node is provided in the **alpo_bridge** package. This bridge allow to broadscast sevevals topics :

- from ROS1 to  ROS2
  - **/joy**  to **/alpo/bridge/joy** (sensors_msg/Joy) :  this topic contains data coming from joystick embedded in the robot
  - **/alpo_driver/ackermann_controller/odom**  to **/alpo/bridge/vehicle_controller/odom** (nav_msgs/Odometry): this topic contains dead reckoning data provided by ROS1 robot controller
  - **/alpo_driver/joint_states**  to **/alpo/bridge/vehicle_controller/joinst_states** (sensor_msgs::msg::JointStates): this topic contains joint states data provided by ROS1 robot controller
- from ROS2 to ROS1
  - **/alpo/bridge/vehicle_controller/cmd_steer** to  **/auto/cmd_steer** (ackerman_msgs/AckermanDrive): this topic is used to send ackermann command to the robot 

It 's easy to launch the bridge_node like this:

```console
ros2 run alpo_bridge alpo_bridge
```

Note that the bridge will be only compile  when ROS1 and ROS2 are installed together. 

# 3 Bringup package

## 3.1 Base launch file ##

{: style="text-align: justify"}
The **alpo_base.launch.py** file, located in the **alpo_bringup/launch** directory, is responsible for starting the controller manager, the robot's controller and a command multiplexer. The configuration of controller manager and robot controller  are defined respectively in the **controller_manager.yam**l and **mobile_base_controller.yaml** files, respectively, which are located in the **alpo_bringup/config** directory. 

 By default, **MobileBaseController2FWS4WD** and  **MobileBaseController2FWS2RWD** controllers are used to control for 4WD (fat) and 2WD (slim) ALPOs, respectively. These two controllers are provided in the **romea_mobile_base_controller** package.  

It is possible to launch these nodes via command line:

```console
ros2 launch alpo_bringup alpo_base_launch.py mode:=simulation robot_namespace:=alpo robot_model:=fat base_name:=base
```

where:
- ***mode*** (choices: ***simulation*** or ***live***) is the demonstration mode,   

- ***robot_model*** (choices: ***fat*** or ***slim***) is the model of the robot, ***fat*** for 4WD ALPO and ***slim*** for 2WD ALPO

- ***robot_namespace*** (default: ***alpo***) is the main ros namespace where all alpo nodes are launched   

- ***base_name*** (default: ***base***) is the ros sub-namespace in which controller nodes are launched

  

## 3.2 Teleop launch file ##

The **alpo_teleop.launch.py** file located in the **bringup/launch** directory, is used to execute an **one_axle_steering_teleop_node** provided in **romea_teleop_drivers** package in order to control ALPO robot's motion. It is possible to launch teleop node via command line:

```console
ros2 launch alpo_bringup alpo_teleop.launch.py robot_model:=fat joystick_type:=xbox joystick_driver:=joy joystic_topic:=joystick/joy teleop_configuration_file_path:=/path_to_file/teleop.yaml
```

where:

***robot_model*** (choices: ***fat*** or ***slim***) specifies of the robot, fat for 4WD ALPO and slim for 2WD ALPO

- ***joystick_type*** (choices: ***xbox*** or ***dualshock4***) specifies the type of joystick
- ***joystick_driver*** (choices: ***joy*** or ***ds4_driver***, default: ***joy***) specifies the ROS2 driver package used to control the joystick
- ***joystic_topic*** (default: **joystick/joy**) specifies the name of the output topic of the  joystick node 
- ***teleop_configuration_file_path*** specifies the absolute path of teleoperation configuration file 

Default teleop configuration file can be found into **config** directory of **alpo_description** package and joystick mapping can be found into files located in the **config** directory of **romea_teleop_description** package.



## 3.3 Test launch file

The **alpo_test.launch.py** file located in the **bringup/launch** directory, is used to test robot control pipeline in live or simulation context. The following nodes are launched  : controller_manger, robot controller , joystick node and teleop node in using a xbox joystick.  

```console
ros2 launch alpo_bringup alpo_test.launch.py robot_model:=fat mode:=simulation
```

where:

- ***mode*** (choices: ***simulation*** or ***live***) is the demonstration mode,   

- ***robot_model*** (choices: ***fat*** or ***slim***) is the model of the robot, ***fat*** for 4WD ALPO and ***slim*** for 2WD ALPO

  

# 4 Description package:



# 5 Hardware package:
