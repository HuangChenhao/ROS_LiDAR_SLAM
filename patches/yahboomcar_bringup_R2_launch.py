from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    urdf_path = get_package_share_path('yahboomcar_description')
    model_path = urdf_path / 'urdf/yahboomcar_R2.urdf.xacro'
    model_arg = DeclareLaunchArgument('model', default_value=str(model_path))
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    imu_config = os.path.join(
        get_package_share_directory('yahboomcar_bringup'), 'param', 'imu_filter_param.yaml')

    robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}], output='screen')
    driver = Node(
        package='yahboomcar_bringup', executable='Ackman_driver_R2', name='driver_node',
        output='screen', respawn=True, respawn_delay=2.0)
    odometry = ExecuteProcess(
        cmd=['python3', '/root/rosmaster_tools/rosmaster_odom.py'],
        output='screen', respawn=True, respawn_delay=2.0)
    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        parameters=[imu_config], output='screen', respawn=True, respawn_delay=2.0)
    ekf = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('robot_localization'), 'launch'),
        '/ekf_x1_x3_launch.py']))
    joy = Node(
        package='yahboomcar_ctrl', executable='yahboom_joy_R2', name='joy_ctrl',
        output='screen', respawn=True, respawn_delay=2.0)

    # The vendor joint_state_publisher was removed: it duplicated /joint_states
    # while the hardware driver already publishes steering and wheel state.
    return LaunchDescription([model_arg, robot_state, driver, odometry, imu_filter, ekf, joy])
