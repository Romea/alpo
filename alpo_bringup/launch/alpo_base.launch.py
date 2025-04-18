# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)

    if "replay" in mode:
        return []

    if mode == "simulation":
        mode += "_gazebo_classic"

    base_name = LaunchConfiguration("base_name").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)

    if robot_namespace:
        controller_manager_name = "/" + robot_namespace + "/base/controller_manager"
        robot_prefix = robot_namespace + "_"
    else:
        controller_manager_name = "/base/controller_manager"
        robot_prefix = ""

    base_description_yaml_file = (
        get_package_share_directory("alpo_description")
        + "/config/alpo_"
        + robot_model
        + ".yaml"
    )

    controller_manager_yaml_file = (
        get_package_share_directory("alpo_bringup") + "/config/controller_manager.yaml"
    )

    base_controller_yaml_file = (
        get_package_share_directory("alpo_bringup")
        + "/config/mobile_base_controller.yaml"
    )

    base_ros2_control_description_file = "/tmp/"+robot_prefix+base_name+"_ros2_control.urdf"
    with open(base_ros2_control_description_file, "r") as f:
        base_ros2_control_description = f.read()

    # alpo_bridge = Node(
    #     condition=LaunchConfigurationEquals("mode", "live"),
    #     package="alpo_bridge",
    #     executable="alpo_bridge",
    #     output="screen",
    # )

    controller_manager = Node(
        condition=IfCondition(PythonExpression(["'gazebo' not in '", mode, "'"])),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": base_ros2_control_description},
            controller_manager_yaml_file
        ],
        # output="screen",
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_mobile_base_controllers"),
                        "launch",
                        "mobile_base_controller.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "joints_prefix": robot_prefix,
            "controller_name": "mobile_base_controller_" + robot_model,
            "controller_manager_name": controller_manager_name,
            "base_description_yaml_filename": base_description_yaml_file,
            "base_controller_yaml_filename": base_controller_yaml_file,
        }.items(),
    )

    cmd_mux = Node(
        package="romea_cmd_mux",
        executable="cmd_mux_node",
        name="cmd_mux",
        parameters=[{"topics_type": "romea_mobile_base_msgs/OneAxleSteeringCommand"}],
        remappings=[("~/out", "controller/cmd_one_axle_steering")],
        # output="screen",
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                PushRosNamespace(robot_namespace),
                PushRosNamespace(base_name),
                # alpo_bridge,
                controller_manager,
                controller,
                cmd_mux,
            ]
        ),
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_model"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="alpo")
    )

    declared_arguments.append(
        DeclareLaunchArgument("base_name", default_value="base")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
