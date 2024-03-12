# 1 Overview #



# 2 Base launch file #

The alpo_base.launch.py file located in the launch directory is used to start a controller manager, the robot's controller and a command multiplexer. The controller manager and robot controller configurations are defined respectively in the controller_manager.yaml and mobile_base_controller.yaml files located in the config directory. By default, MobileBaseController2FWS4WD and  MobileBaseController2FWS2RWD controllers are respectively used to control for 4WD (fat) and 2WD (slim) alpos. These two controllers are provided in the romea_mobile_base_controller package. 

It is possible to launch these nodes via command line:

```console
ros2 launch alpo_bringup alpo_base_launch.py mode:=simulation robot_namespace:=alpo robot_model:=fat base_name:=base
```

where:
- *mode (choices: simulation or live)* is the demonstration mode,   
- *robot_model (choices: fat or slim)* is the model of the robot, fat for 4WD alpo and slim for 2WD alpo
- *robot_namespace (default: alpo)* is the main ros namespace where all alpo nodes are launched   
- *base_name* (default: base) is the ros sub-namespace in which controller nodes are launched

# 3 Teleop launch file #

The alpo_teleop.launch.py file located in the launch directory is used to execute an one_axle_steering_teleop_node provided in romea_teleop_drivers package in order to control alpo robot motion. It possible to launch teleop node via command line:

```console
ros2 launch alpo_bringup alpo_teleop.launch.py robot_model:=fat joystick_type:=xbox joystick_driver:=joy joystic_topic:=joystick/joy teleop_configuration_file_path:=/path_to_file/teleop.yaml
```

where:
- *robot_model (choices: fat or slim)* is the model of the robot, fat for 4WD alpo and slim for 2WD alpo
- *joystick_type(choices: xbox or dualshock4)* is the type of joystick
- *joystick_driver(choices: joy or ds4_driver, default: joy)* is the name of ros2 driver pkg used to control joystick
- *joystic_topic(default: joystick/joy)* is the name of output topic of joystick node 
- *teleop_configuration_file_path* is the absolute path of teleoperation configuration file 

Default teleop configuration file can be found into config directory of alpo_description package and joystick mapping can be found into files located in the config directory of romea_teleop_description package.
