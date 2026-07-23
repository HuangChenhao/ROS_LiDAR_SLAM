import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_path = get_package_share_directory('yahboomcar_nav')
    configuration_directory = LaunchConfiguration(
        'configuration_directory', default=os.path.join(package_path, 'params'))
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='rosmaster_carto.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period = LaunchConfiguration('publish_period_sec', default='1.0')

    cartographer = Node(
        package='cartographer_ros', executable='cartographer_node',
        name='cartographer_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        remappings=[('scan', '/scan'), ('odom', '/odom'), ('imu', '/imu/data')])

    occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period,
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('configuration_directory', default_value=configuration_directory),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period),
        cartographer,
        occupancy_grid,
    ])
