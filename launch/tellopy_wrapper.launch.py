#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, PythonExpression, PathJoinSubstitution, EnvironmentVariable

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'debug', default_value = 'false',
        description='Run spawner with debug log level'
    ))

    # Conditionally set the log level using PythonExpression
    log_level = PythonExpression([
        "'debug' if '", LaunchConfiguration('debug'), "' == 'true' else 'info'"
    ])

    ld.add_action(
        Node(
            name='tellopy_wrapper',
            package='mrs_uav_dji_tello_api',
            executable='tellopy_wrapper.py',
            namespace="uav1",
            output="screen",
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'uav_name': "uav1"},
                ],
            remappings=[
              # out
              ("~/battery_out", "~/battery"),
              ("~/height_out", "~/height"),
              ("~/pose_out", "~/pose"),
              ("~/twist_out", "~/twist"),
              ("~/armed_out", "~/armed"),
              ("~/image_raw_out", "~/rgb/image_raw"),
              ("~/camera_info_out", "~/rgb/camera_info"),
              ("~/imu_out", "~/imu"),

              # in
              ("~/arm_in", "~/arm"),
              ("~/throw_in", "~/throw"),
              ("~/zero_in", "~/zero"),
              ("~/cmd_in", "~/cmd"),
            ],
            )
        )

    return ld
