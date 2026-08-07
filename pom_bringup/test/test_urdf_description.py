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


import subprocess
import xml.etree.ElementTree as ET

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory


def urdf_xml(mode, model):

    exe = get_package_prefix("pom_bringup") + "/lib/pom_bringup/generate_urdf_description.py"

    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "mode:" + mode,
                "robot_model:" + model,
                "base_name:base",
                "robot_namespace:robot",
            ],
            encoding="utf-8",
        )
    )


def ros2_control_xml(mode, model):

    exe = (
        get_package_prefix("pom_bringup")
        + "/lib/pom_bringup/generate_ros2_control_description.py"
    )

    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "mode:" + mode,
                "robot_model:" + model,
                "base_name:base",
                "robot_namespace:robot",
            ],
            encoding="utf-8",
        )
    )


def test_footprint_link_name():
    assert urdf_xml("live", "basic").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert ros2_control_xml("live", "4x4").find(
        "ros2_control/hardware/plugin"
    ).text == "pom_hardware/PomHardware2FWS4WD"

    assert ros2_control_xml("live", "basic").find(
        "ros2_control/hardware/plugin"
    ).text == "pom_hardware/PomHardware2FWS2RWD"

    assert ros2_control_xml("simulation_gazebo", "basic").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface"

    assert ros2_control_xml("simulation_gazebo", "4x4").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface"

    assert ros2_control_xml("simulation_gazebo_classic", "basic").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface2FWS2RWD"

    assert ros2_control_xml("simulation_gazebo_classic", "4x4").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface2FWS4WD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation_gazebo_classic", "basic").find("gazebo/plugin/parameters").text
        == get_package_share_directory("pom_bringup") + "/config/controller_manager.yaml"
    )
