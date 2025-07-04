# pom_bringup #

## 1 Launch files ##

The pom_bringup package provides a suite of launch files enabling both manual and automated control of Pom robots, either in real-time or in simulation, including their lifting systems. Two robot models are supported (see figure below) are Pom Basic and Pom  4x4. The Pom Basic model is equipped with a single front lifting system and two rear driving wheels, while the Pom4x4 model features two lifting systems, one at the front and one at the rear, along with four driving wheels.

| ![Controller mapping](doc/alpo_slim.jpg) | ![Controller mapping](doc/alpo_fat.jpg) |
| ---------------------------------------- | --------------------------------------- |

### 1.1 Base launch file ###

The **pom_base.launch.py** file, located in the launch directory, is responsible for starting the controller manager, the robot's controller, and a command multiplexer. The configuration of the controller manager and the robot controller are defined in the **controller_manager.yaml** and **mobile_base_controller.yaml** files, respectively, which are located in the config directory.

By default, **MobileBaseController2FWS4WD** and **MobileBaseController2FWS2RWD** controllers are used to control the 4WD (4x4) and 2WD (basic) Pom robots, respectively. These two controllers are provided by the **romea_mobile_base_controller** package. 

You can launch these nodes via command line:

```console
ros2 launch pom_bringup pom_base_launch.py mode:=simulation robot_namespace:=pom robot_model:=4x4 base_name:=base
```

where:
- ***mode*** (choices: ***simulation*** or ***live***) defines the demonstration mode.  
- ***robot_model*** (choices: ***4x4*** or ***basic***) defines the robot model, with `4x4` for 4WD Alpo and `basic` for 2WD Alpo.
- ***robot_namespace*** (default: ***pom***) sets the main ROS namespace where all Alpo nodes are launched. 
- ***base_name*** (default: ***base***) sets the ROS sub-namespace in which controller nodes are launched
`

### 1.2 Teleop launch file ###

The **pom_teleop.launch.py** file, located in the launch directory, is used to execute the **one_axle_steering_teleop_node** provided by the **romea_mobile_base_teleop** package to control the motion of the Alpo robot. You can launch the teleop node via the command line:

```console
ros2 launch pom_bringup pom_teleop.launch.py robot_model:=4x4 joystick_type:=xbox joystick_driver:=joy joystic_topic:=joystick/joy teleop_configuration_file_path:=/path_to_file/teleop.yaml
```

where:

- ***robot_model*** (choices: ***4x4*** or ***basic***) specifies of the robot, fat for 4WD Pom and slim for 2WD Pom

- ***joystick_type*** (choices: ***xbox*** or ***dualshock4***) specifies the type of joystick
- ***joystick_driver*** (choices: ***joy*** or ***ds4_driver***, default: ***joy***) defines the ROS2 driver package used to control the joystick
- ***joystic_topic*** (default: **joystick/joy**) defines the name of the output topic of the  joystick node 
- ***teleop_configuration_file_path*** specifies the absolute path of teleoperation configuration file 

The default teleop configuration file can be found in the config directory of the **pom_description** package, and the joystick mapping can be found in the config directory of the **romea_mobile_base_teleop** package. To move the robot you need to keep your finger pressed on either the slow mode or turbo mode button and use the sticks to control the speed and direction of the robot.

![Controller mapping](doc/teleop.jpg)

TODO(Vincent & Jean) Implement control

## 2.3 Test launch file

The **pom_test.launch.py** file, located in the launch directory, is used to test the robot control pipeline in both live and simulation contexts. The following nodes are launched: controller manager, robot controller, joystick node, and teleop node using an Xbox joystick.

```console
ros2 launch pom_bringup pom_test.launch.py robot_model:=4x4 mode:=simulation
```

where:

- ***mode*** (choices: ***simulation*** or ***live***) defines the demonstration mode,   
- ***robot_model*** (choices: ***4x4*** or ***basic***) defines the model of the robot, ***4x4*** for 4WD Pom and ***basic*** for 2WD Pom

Below, you can see the ROS pipeline when selecting the simulation mode with the Alpo Fat robot. In live mode, the pipeline remains the same, except that the **gazebo_ros2_controller_manager** is replaced by a standard **ros2_controller_manager**. Additionally, when selecting the Pom Bacis , the **mobile_base_controller_basic**` is launched instead of the **mobile_base_controller_4x4**.

![Controller mapping](doc/test_pipeline.png)

# 2 URDF description:

You can generate the URDF description of the Pom robot using the **urdf_description.py** executable located in the scripts directory.

```console
ros2 run pom_bringup urdf_description.py robot_model:4x4 mode:simulation base_name:base robot_namespace:pom > pom.urdf
```

where:

- ***base_name***  defines the name of robot mobile base  
- ***mode*** (choices: ***simulation*** or ***live***) defines the demonstration mode,  
- ***robot_model*** (choices: ***4x4*** or ***basic***) defines the model of the robot, ***4x4*** for 4WD Pom and ***basic*** for 2WD Pom
- **robot_namespace** is the namespace in which the ROS2 nodes are launched. It is also used as a prefix for link and joints of the mobile base. 
