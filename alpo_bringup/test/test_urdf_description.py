# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import subprocess

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

import xml.etree.ElementTree as ET


def urdf_xml(mode, model):

    exe = get_package_prefix("alpo_bringup") + "/lib/alpo_bringup/urdf_description.py"

    return ET.fromstring(
        subprocess.check_output(
            [exe, "mode:" + mode, "robot_model:" + model, "robot_namespace:robot"],
            encoding="utf-8",
        )
    )


def test_footprint_link_name():
    assert urdf_xml("live", "slim").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert urdf_xml("live", "fat").find(
        "ros2_control/hardware/plugin"
    ).text == "alpo_hardware/AlpoHardware2FWS4WD"

    assert urdf_xml("live", "slim").find(
        "ros2_control/hardware/plugin"
    ).text == "alpo_hardware/AlpoHardware2FWS2RWD"

    assert urdf_xml("simulation", "slim").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface2FWS2RWD"

    assert urdf_xml("simulation", "fat").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface2FWS4WD"


def test_controller_filename_name():

    assert (
        urdf_xml("simulation", "slim").find("gazebo/plugin/parameters").text
        == get_package_share_directory("alpo_bringup")
        + "/config/controller_manager.yaml"
    )
