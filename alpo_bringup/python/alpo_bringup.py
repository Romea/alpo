#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import alpo_description

def urdf_description(prefix, mode, model):

    controller_manager_yaml_file = (
        get_package_share_directory("alpo_bringup")
        + "/config/controller_manager.yaml"
    )

    return alpo_description.urdf(prefix, mode, model, controller_manager_yaml_file)
