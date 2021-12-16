import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory("obstacle_detector"),
                'resources', 'obstacle_detector.rviz'),
            description='Full path to the rviz config file'),

    # Launch rviz
    Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_file],
        output='screen')
    ])
