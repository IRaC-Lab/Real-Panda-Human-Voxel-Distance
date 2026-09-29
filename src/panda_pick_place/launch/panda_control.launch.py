"""Bring up the Panda (real or mock) with ros2_control and the Franka Hand.

    ros2 launch panda_pick_place panda_control.launch.py robot_ip:=172.16.0.2
    ros2 launch panda_pick_place panda_control.launch.py use_fake_hardware:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # Pick the controller file here: a LaunchConfiguration is always truthy in a
    # plain Python `if`, so it has to be resolved against the launch context.
    fake = use_fake_hardware.perform(context).lower() in ('true', '1')
    config_dir = os.path.join(get_package_share_directory('panda_pick_place'), 'config')
    controllers = os.path.join(
        config_dir, 'controllers_fake.yaml' if fake else 'controllers_real.yaml')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(get_package_share_directory('franka_description'), 'robots',
                     'panda_arm.urdf.xacro'),
        ' hand:=true robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'source_list': ['franka/joint_states', 'panda_gripper/joint_states'],
                         'rate': 30}],
        ),
        Node(
            package='franka_control2',
            executable='franka_control2_node',
            parameters=[{'robot_description': robot_description}, controllers],
            remappings=[('joint_states', 'franka/joint_states')],
            output='screen',
            on_exit=Shutdown(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster'],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['panda_arm_controller'],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={'robot_ip': robot_ip,
                              'use_fake_hardware': use_fake_hardware}.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.2',
                              description='Hostname or IP address of the Panda.'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='Use mock hardware instead of a real Panda.'),
        OpaqueFunction(function=_launch_setup),
    ])
