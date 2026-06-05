# pom_description

## 1) Overview

`pom_description` provides the robot-specific description layer for the POM mobile bases.

It extends `romea_mobile_base_description` with the concrete configuration, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate POM robots.

The supported POM variants are:

| Variant | Configuration file | Mobile base architecture | Command type |
|---|---|---|---|
| `basic` | `config/pom_basic.yaml` | `2FWS2RWD` | `one_axle_steering` |
| `4x4` | `config/pom_4x4.yaml` | `2FWS4WD` | `one_axle_steering` |

Both variants use front wheel steering. The `basic` variant has two rear driving wheels, while the `4x4` variant has four driving wheels.

## 2) Robot configuration

The `config/` directory contains:

| File | Purpose |
|---|---|
| `pom_basic.yaml` | full POM basic mobile base configuration |
| `pom_4x4.yaml` | full POM 4x4 mobile base configuration |
| `teleop.yaml` | default one-axle steering teleoperation configuration |

Each robot configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture (`2FWS2RWD` or `2FWS4WD`);
* geometry, wheel dimensions and chassis bounding box;
* front wheel steering command and feedback information;
* rear wheel speed command and feedback information for `basic`;
* wheel speed command and feedback information for `4x4`;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/pom_basic.urdf.xacro` | entry point for the `basic` variant |
| `urdf/pom_4x4.urdf.xacro` | entry point for the `4x4` variant |
| `urdf/pom.xacro` | common POM mobile base macro |
| `urdf/pom.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `urdf/visual/` | visual Xacro fragments for chassis, arms, wheels and half legs |
| `meshes/` | chassis, wheel, half-leg and rollbar visual meshes |

The common POM macro reuses the `base2FWS2RWD.chassis.xacro` or `base2FWS4WD.chassis.xacro` template from `romea_mobile_base_description` and specializes it with POM geometry, link names, joint names and visual meshes.

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/pom_basic.ros2_control.urdf.xacro` | `ros2_control` entry point for the `basic` variant |
| `ros2_control/pom_4x4.ros2_control.urdf.xacro` | `ros2_control` entry point for the `4x4` variant |
| `ros2_control/pom.ros2_control.xacro` | common POM `ros2_control` macro |

Depending on the selected mode, the `ros2_control` description selects:

| Variant | Mode | Hardware plugin |
|---|---|---|
| `basic` | `live` | `pom_hardware/PomHardware2FWS2RWD` |
| `4x4` | `live` | `pom_hardware/PomHardware2FWS4WD` |
| `basic` | `simulation`, `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface2FWS2RWD` |
| `4x4` | `simulation`, `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface2FWS4WD` |
| `basic` | `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface2FWS2RWD` |
| `4x4` | `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface2FWS4WD` |
| `basic` | `simulation_4dv`, `simulation_isaac` | `romea_mobile_base_simulation/GenericSimulationSystemInterface2FWS2RWD` |
| `4x4` | `simulation_4dv`, `simulation_isaac` | `romea_mobile_base_simulation/GenericSimulationSystemInterface2FWS4WD` |

## 4) Python API

The installed Python module provides helper functions used by `pom_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file(robot_model)` | returns the configuration file path for `basic` or `4x4` |
| `get_specifications_configuration(robot_model)` | loads the full robot configuration |
| `get_configuration(robot_model)` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the POM URDF description |
| `generate_ros2_control_description(...)` | generates the POM `ros2_control` description |

Example:

```python
from pom_description import get_configuration

configuration = get_configuration("4x4")
```

When `mode` is set to `simulation`, the Python API maps it to `simulation_gazebo_classic` before generating the URDF or `ros2_control` description.

## 5) Relation with other packages

`pom_description` is the POM specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic `2FWS2RWD` and `2FWS4WD` description and `ros2_control` templates;
* `pom_description` provides the POM configurations, meshes and Xacro specialization;
* `pom_hardware` provides the live `ros2_control` hardware plugins;
* `pom_bringup` uses this package to generate configuration, URDF and `ros2_control` artifacts for live and simulation modes.
