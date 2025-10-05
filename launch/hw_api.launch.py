#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_dji_tello_api"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace='hw_api'

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
    )

    # #} end of custom_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "true"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # the first one has the priority
    configs = [
        this_pkg_path + '/config/tello_api.yaml',
        get_package_share_directory("mrs_uav_hw_api") + "/config/hw_api.yaml",
    ]

    ld.add_action(ComposableNodeContainer(

        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",

        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],

        composable_node_descriptions=[

            ComposableNode(

                package="mrs_uav_hw_api",
                plugin='mrs_uav_hw_api::HwApiManager',
                namespace=uav_name,
                name='hw_api',
                parameters=[
                    {"uav_name": uav_name},
                    {"topic_prefix": ["/", uav_name]},
                    {"use_sim_time": use_sim_time},
                    {"configs": configs},
                    {'custom_config': custom_config},
                ],

                remappings=[
                  ("~/status_in", "tellopy_wrapper/state"),
                  ("~/pose_in", "tellopy_wrapper/pose"),
                  ("~/twist_in", "tellopy_wrapper/twist"),
                  ("~/battery_in", "tellopy_wrapper/battery"),
                  ("~/arm_out", "tellopy_wrapper/arm"),
                  ("~/cmd_out", "tellopy_wrapper/cmd"),
                  ("~/armed_in", "tellopy_wrapper/armed"),
                  ("~/height_in", "tellopy_wrapper/height"),
                  ("~/imu_in", "tellopy_wrapper/imu"),
                ],
            )

        ],

    ))

    return ld
