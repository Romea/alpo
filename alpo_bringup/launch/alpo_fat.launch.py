from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xml.etree.ElementTree as ET
import re


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    joystick_type = LaunchConfiguration("joystick_type").perform(context)
    urdf_description = LaunchConfiguration("urdf_description").perform(context)

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("alpo_bringup"), "launch", "alpo.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "mode": mode,
            "robot_namespace": robot_namespace,
            "robot_model": "fat",
            "joystick_type": joystick_type,
            "urdf_description": urdf_description,
        }.items(),
    )

    return [base]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="simulation"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="alpo")
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_type", default_value="xbox")
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_gazebo", default_value="True")
    )

    urdf_description = Command(
        [
            ExecutableInPackage("urdf_description.py", "alpo_bringup"),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " mode:",
            LaunchConfiguration("mode"),
            " robot_model:fat",
        ]
    )

    declared_arguments.append(
        DeclareLaunchArgument("urdf_description", default_value=urdf_description)
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )