# pom

## Overview

`pom` groups the ROS2 packages that describe, launch and control the POM mobile bases in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `pom` | Metapackage that groups the POM ROS2 packages. |
| `pom_description` | Robot-specific description layer for POM variants, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `pom_bringup` | Main integration entry point for generating POM configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `pom_hardware` | Live `ros2_control` hardware plugin for POM mobile bases. |
| `pom_bridge` | ROS1 / ROS2 bridge used by live POM robots to exchange commands and feedback with the low-level controller. |

## Usage

In most cases, start with `pom_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when a POM model is selected from a mobile base meta-description.

The POM stack is a robot-specific specialization of `romea_mobile_base`. The supported variants are `basic` and `4x4`; both use the `one_axle_steering` command type, with `2FWS2RWD` for `basic` and `2FWS4WD` for `4x4`. `pom_description` provides the concrete geometry and generated descriptions, `pom_hardware` provides the live hardware implementation, `pom_bridge` connects the ROS2 stack to the embedded ROS1 interface, and `pom_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `pom` project was developed by Jean Laneurit in the context of the TIARA ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
