import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory("obstacle_detector"),
                'params', 'tracker.yaml'),
            description='Full path to the ostacle tracker parameters file to use for all launched nodes'),

        Node(
            parameters=[
              params_file,
              { "use_sim_time": use_sim_time },
            ],
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            name='obstacle_extractor',
            output='screen'
        ),

        Node(
            parameters=[
              params_file,
              { "use_sim_time": use_sim_time },
            ],
            package='obstacle_detector',
            executable='obstacle_tracker_node',
            name='obstacle_tracker',
            output='screen'
        ),
    ])
