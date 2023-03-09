# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


# import pytest
import xml.etree.ElementTree as ET
from alpo_description import urdf


def urdf_xml(mode, model):
    prefix = "robot_"
    ros_namespace = "/robot"
    controller_conf_yaml_file = mode + "_" + model + "_controller.yaml"
    return ET.fromstring(urdf(prefix, mode, model, controller_conf_yaml_file, ros_namespace))


def test_footprint_link_name():
    assert urdf_xml("live", "slim").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert (
        urdf_xml("live", "fat").find("ros2_control/hardware/plugin").text
        == "alpo_hardware/AlpoHardware2FWS4WD"
    )

    assert (
        urdf_xml("live", "slim").find("ros2_control/hardware/plugin").text
        == "alpo_hardware/AlpoHardware2FWS2RWD"
    )

    assert (
        urdf_xml("simulation", "fat").find("ros2_control/hardware/plugin").text
        == "romea_mobile_base_gazebo/GazeboSystemInterface2FWS4WD"
    )

    assert (
        urdf_xml("simulation", "slim").find("ros2_control/hardware/plugin").text
        == "romea_mobile_base_gazebo/GazeboSystemInterface2FWS2RWD"
    )


def test_controller_filename_name():

    assert (
        urdf_xml("simulation", "slim").find("gazebo/plugin/parameters").text
        == "simulation_slim_controller.yaml"
    )
