from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from mrs_lib.remappings_custom_config_parser import RemappingsCustomConfigParser

import launch

import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('flame_ros'),
            'config',
            'flame_component_sim.yaml'
        ]),
        description='Path to the FLAME configuration file'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'cam_type',
        default_value='rgb',
        description='Path to the FLAME configuration file'
    ))

    node = ComposableNode(
        package='flame_ros',
        plugin='flame_ros::FlameRos',
        name='flame',
        namespace=uav_name,
        parameters=[
            {"use_sim_time": True},
            LaunchConfiguration('custom_config')
        ],
        remappings=[
            # subscribers
            ('~/image_in', [LaunchConfiguration('cam_type'), '/image_raw']),
            ('~/camera_info', [LaunchConfiguration('cam_type'), '/camera_info']),
            # publishers
            ('~/mesh_out', '~/mesh'),
            ('~/cloud_out', '~/cloud'),
            ('~/stats_out', '~/stats'),
            ('~/nodelet_stats_out', '~/nodelet_stats'),
            # image publishers
            ('~/debug/wireframe', '~/debug/wireframe'),
            ('~/debug/features', '~/debug/features'),
            ('~/debug/directions', '~/debug/directions'),
            ('~/debug/matches', '~/debug/matches'),
            ('~/debug/normals', '~/debug/normals'),
            ('~/debug/idepthmap', '~/debug/idepthmap'),
            ('~/idepth_registered/image_rect', '~/idepth_registered/image_rect'),
            ('~/depth_registered/image_rect', '~/depth_registered/image_rect'),
            ('~/depth_registered_raw/image_rect', '~/depth_registered_raw/image_rect'),
            ]
    )

    ld.add_action(RemappingsCustomConfigParser(node, LaunchConfiguration('custom_config')))

    # #{ container

    container = ComposableNodeContainer(
        name='flame_container',
        namespace=uav_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        #prefix='xterm -e gdb -ex run --args',
        # prefix='gdb -ex run --args',
        # prefix='valgrind --tool=massif',
        composable_node_descriptions=[node],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
            {'use_sim_time': True},
        ],
    )

    ld.add_action(container)

    # #} end of container

    return ld
