# pom_bridge

## 1) Overview

`pom_bridge` provides the ROS1 / ROS2 communication bridge used to operate POM robots from the ROS2 ROMEA stack.

The POM low-level controller exposes a ROS1 interface on the robot. This package runs a process that owns both a ROS1 node and a ROS2 node, then forwards the messages required by the live POM hardware plugin and teleoperation stack.

It bridges:

* ROS2 Ackermann steering commands to the ROS1 POM low-level controller;
* ROS2 implement commands to the ROS1  front and rear actuator interfaces;
* ROS1 odometry, joint states, joystick and battery state feedback back to ROS2.

`pom_hardware` uses this bridge in live mode instead of communicating directly with the low-level controller.

## 2) Runtime behavior

At runtime, `pom_bridge_node`:

1. creates a ROS2 node named `bridge`;
2. creates a ROS1 node named `pom_bridge`;
3. optionally overrides the ROS1 master and local ROS1 IP address to connect to the robot network;
4. starts a ROS1 asynchronous spinner;
5. creates ROS1 publishers and subscribers;
6. creates ROS2 publishers and subscribers;
7. copies compatible message fields between ROS1 and ROS2 message types.

By default, the node searches for a local network interface in the `192.168.100.*` range and uses the robot ROS1 master:

```text
http://192.168.100.1:11311
```

This behavior is controlled by the ROS2 parameter:

| Parameter | Default | Purpose |
|---|---|---|
| `override_ros1_master` | `true` | when enabled, configures the ROS1 master and local IP from the POM robot network |

## 3) ROS2 to ROS1 bridge

The following ROS2 topics are subscribed by the bridge and forwarded to the ROS1 POM low-level controller:

| ROS2 topic | ROS2 message | ROS1 topic | ROS1 message |
|---|---|---|---|
| `~/vehicle_controller/cmd_steer` | `ackermann_msgs/msg/AckermannDrive` | `/auto/cmd_steer` | `ackermann_msgs/AckermannDrive` |
| `implement/front/command` | `romea_implement_msgs/msg/Command` | `/cylinder/front/interface/command` | `linak_a36_msgs/CylinderCommand` |
| `implement/rear/command` | `romea_implement_msgs/msg/Command` | `/cylinder/rear/interface/command` | `linak_a36_msgs/CylinderCommand` |

The bridge maps implement commands such as `GO_UP`, `GO_DOWN`, `GO_TO_ANCHOR_LOW` and `GO_TO_ANCHOR_HIGH` to the corresponding ROS1 cylinder commands.

## 4) ROS1 to ROS2 bridge

The following ROS1 topics are subscribed by the bridge and republished on the ROS2 side:

| ROS1 topic | ROS1 message | ROS2 topic | ROS2 message |
|---|---|---|---|
| `/pom_driver/ackermann_controller/odom` | `nav_msgs/Odometry` | `~/vehicle_controller/odom` | `nav_msgs/msg/Odometry` |
| `/pom_driver/joint_states` | `sensor_msgs/JointState` | `~/vehicle_controller/joint_states` | `sensor_msgs/msg/JointState` |
| `/joy` | `sensor_msgs/Joy` | `~/joy` | `sensor_msgs/msg/Joy` |
| `/power_supply/battery_status` | `sensor_msgs/BatteryState` | `~/power_supply/battery_status` | `sensor_msgs/msg/BatteryState` |

The joint state feedback is consumed by `pom_hardware` and converted into `ros2_control` state interfaces. The odometry, joystick and battery topics are made available to the ROS2 stack for monitoring or higher-level behavior.

## 5) Usage

The package installs both the bridge executable and a helper script.

```bash
ros2 run pom_bridge pom_bridge_node
```

The helper script can be used when the ROS1 environment must be sourced before starting the bridge:

```bash
ros2 run pom_bridge pom_bridge
```

## 6) Docker image

The `docker/` directory provides a Docker setup for running the bridge with both ROS1 and ROS2 environments available in the same container.

This is useful when the host system does not provide the ROS1 dependencies required by the POM low-level interface.

Example service:

```yaml
services:
  bridge:
    image: ghcr.io/romea/alpo
    network_mode: host
    environment:
      - ROS_IP=192.168.100.2
      - ROS_MASTER_URI=http://192.168.100.1:11311
      - ROS_DOMAIN_ID=1
    command: >-
      ros2 run pom_bridge pom_bridge
        --ros-args
        -r __ns:=/pom/base
        -p override_ros1_master:=false
```

The image build files and registry update instructions are documented in `docker/README.md`.

## 7) Relation with other packages

`pom_bridge` is part of the live POM control path:

* `pom_hardware` publishes Ackermann commands to the bridge and receives joint states from it;
* `pom_bringup` starts the live control stack that uses `pom_hardware`;
* the POM ROS1 low-level controller remains responsible for the embedded Ackermann control, odometry, joint state feedback and actuator interfaces.

Simulation modes do not use this package.
