# pom_bringup

## 1) Overview

`pom_bringup` connects the POM description, hardware, bridge, simulation and teleoperation packages to the generic `romea_mobile_base_meta_bringup` workflow.

It provides:

* robot-specific generation functions for configuration, URDF and `ros2_control` descriptions;
* launch files for live control, Gazebo simulation and teleoperation;
* controller manager and mobile base controller parameter files.

The POM mobile bases are available in two variants:

| Variant | Mobile base architecture | Controller |
|---|---|---|
| `basic` | `2FWS2RWD` | `romea_mobile_base_controllers/MobileBaseController2FWS2RWD` |
| `4x4` | `2FWS4WD` | `romea_mobile_base_controllers/MobileBaseController2FWS4WD` |

Both variants use the `one_axle_steering` command type.

| POM basic | POM 4x4 |
|---|---|
| ![POM basic](doc/alpo_slim.jpg) | ![POM 4x4](doc/alpo_fat.jpg) |

## 2) Generated artifacts

The Python module `pom_bringup` delegates most generation work to `pom_description` and adds bringup-specific configuration such as the controller manager parameter file.

It provides the functions expected by `romea_mobile_base_meta_bringup`:

| Function | Purpose |
|---|---|
| `get_configuration(robot_model)` | returns the compact mobile base configuration for `basic` or `4x4` |
| `generate_configuration_file(robot_model, extended)` | generates the mobile base configuration file |
| `generate_urdf_description(prefix, mode, base_name, robot_model, ros_prefix)` | generates the POM URDF description |
| `generate_ros2_control_description(prefix, mode, base_name, robot_model)` | generates the POM `ros2_control` description |

The executable scripts in `scripts/` expose these functions from the command line.

The configuration generator writes the compact mobile base configuration used by controllers, teleoperation and launch files. It is derived from `pom_description/config/pom_<robot_model>.yaml`.

```bash
ros2 run pom_bringup generate_configuration_file.py \
  robot_model:4x4 \
  extended:false
```

The URDF generator writes the POM robot description. It contains the architecture-specific link and joint structure, inertial data, collision geometry, visual meshes and the simulator plugin block when a simulation mode is selected.

```bash
ros2 run pom_bringup generate_urdf_description.py \
  robot_namespace:pom \
  robot_model:4x4 \
  base_name:base \
  mode:simulation_gazebo_classic
```

The `ros2_control` generator writes the hardware description consumed by `controller_manager`. It declares the hardware plugin selected by the mode, the geometric hardware parameters and the command/state interfaces for the front steering and wheel spinning joints.

```bash
ros2 run pom_bringup generate_ros2_control_description.py \
  robot_namespace:pom \
  robot_model:4x4 \
  base_name:base \
  mode:live
```

## 3) Launch files

### 3.1) Base launch

`launch/pom_base.launch.py` starts the POM mobile base control stack.

It:

* receives the generated robot URDF and `ros2_control` description from the meta-bringup launch context;
* starts `controller_manager/ros2_control_node` in non-Gazebo modes;
* loads `joint_state_broadcaster`;
* loads `mobile_base_controller_basic` or `mobile_base_controller_4x4` depending on `robot_model`;
* starts `romea_cmd_mux` and remaps its output to `controller/cmd_one_axle_steering`.

Main launch arguments are:

| Argument | Description |
|---|---|
| `mode` | execution mode, such as `live`, `simulation_gazebo` or `simulation_gazebo_classic` |
| `robot_model` | POM variant, either `basic` or `4x4` |
| `robot_namespace` | namespace of the robot |
| `base_name` | namespace of the mobile base, usually `base` |

### 3.2) Teleoperation launch

`launch/pom_teleop.launch.py` starts the mobile base teleoperation stack through `romea_mobile_base_teleop`.

It uses:

* the selected POM robot configuration from `pom_description/config/pom_<robot_model>.yaml`;
* the joystick configuration file, usually selected from the `config/` directory of `romea_joystick_utils` according to the joystick type;
* the teleoperation configuration from `pom_description/config/teleop.yaml` by default.

The teleoperation node publishes `romea_mobile_base_msgs/OneAxleSteeringCommand`, consistent with the command type of both POM variants.

To move the robot, the operator must hold either the slow mode or turbo mode button. The joystick axes then command the longitudinal speed and the steering angle.

Main launch arguments are:

| Argument | Description |
|---|---|
| `mode` | execution mode, used to configure simulation time |
| `robot_model` | POM variant, either `basic` or `4x4` |
| `joystick_topic` | joystick `sensor_msgs/msg/Joy` topic |
| `joystick_configuration_file_path` | joystick configuration file, usually selected from `romea_joystick_utils/config/` |
| `teleop_configuration_file_path` | teleoperation configuration file, defaulting to `pom_description/config/teleop.yaml` |

![POM teleoperation mapping](doc/teleop.jpg)

### 3.3) Gazebo launch

`launch/pom_gazebo.launch.py` starts a Gazebo or Gazebo Classic simulation and spawns the selected POM entity from the generated URDF.

It supports:

* `simulation_gazebo`, using `ros_gz_sim` and `gz_ros2_control`;
* `simulation_gazebo_classic`, using `gazebo_ros` and `gazebo_ros2_control`.

The `ros2_control` hardware plugin used in simulation is selected by `pom_description` from the generated `mode` and `robot_model`.

Main launch arguments are:

| Argument | Description |
|---|---|
| `mode` | simulation mode, usually `simulation_gazebo` or `simulation_gazebo_classic` |
| `robot_model` | POM variant, either `basic` or `4x4` |
| `robot_namespace` | namespace of the robot and simulation entity |
| `base_name` | namespace of the mobile base, usually `base` |

### 3.4) Implement teleoperation launch

`launch/pom_implement_teleop.launch.py` starts the teleoperation stack used to command POM implement actuators.

This launch file is separate from the mobile base teleoperation launch because it controls the implement side of the robot, not the one-axle steering motion controller.

### 3.5) Test launch

`launch/pom_test.launch.py` starts a compact test setup with:

* the selected POM simulation when the selected mode contains `simulation`;
* the POM base launch;
* the POM teleoperation launch;
* a joystick node using the selected joystick model.

In simulation mode, the controller manager is provided by the Gazebo integration. In live mode, the base launch starts the standard `controller_manager/ros2_control_node`.

Main launch arguments are:

| Argument | Description |
|---|---|
| `mode` | execution mode, usually `simulation_gazebo_classic` for this test setup |
| `robot_model` | POM variant, either `basic` or `4x4` |
| `joystick_model` | joystick model used to select the default joystick configuration, such as `microsoft_xbox` or `sony_dualshock4` |

The following diagram gives an overview of the control pipeline started by this test launch file.

![POM test pipeline](doc/test_pipeline.png)

## 4) Configuration files

The `config/` directory contains:

| File | Purpose |
|---|---|
| `controller_manager.yaml` | declares `joint_state_broadcaster`, `MobileBaseController2FWS2RWD` and `MobileBaseController2FWS4WD` |
| `mobile_base_controller.yaml` | provides common runtime parameters for the mobile base controllers |

The robot geometry, inertia, joint names and teleoperation defaults are stored in `pom_description/config/`.

## 5) Relation with the meta-bringup workflow

`pom_bringup` is the robot-specific extension used when a mobile base meta-description selects:

```yaml
configuration:
  manufacturer: sabi-agri
  model: pom
  version: 4x4
```

or:

```yaml
configuration:
  manufacturer: sabi-agri
  model: pom
  version: basic
```

In that workflow:

* `romea_mobile_base_meta_bringup` reads the mobile base meta-description;
* `pom_bringup` generates POM-specific configuration, URDF, `ros2_control` and launch artifacts;
* `pom_description` provides the concrete robot model;
* `pom_hardware` and `pom_bridge` are used in `live` mode;
* `romea_mobile_base_gazebo` or `romea_mobile_base_gazebo_classic` is used in Gazebo simulation modes;
* `romea_mobile_base_teleop` starts the matching one-axle steering teleoperation node.
