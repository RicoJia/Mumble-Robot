#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare launch arguments
    no_rviz_arg = DeclareLaunchArgument(
        'no_rviz',
        default_value='false',
        description='Flag to disable RViz'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='-0.0',
        description='X position to spawn the robot'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y position to spawn the robot'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.05',
        description='Z position to spawn the robot'
    )

    # Launch configuration variables
    no_rviz = LaunchConfiguration('no_rviz')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')

    # Set GAZEBO_MODEL_PATH environment variable
    gazebo_model_path = PathJoinSubstitution([
        FindPackageShare('ros2_dream_base_description')
    ])
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[LaunchConfiguration('GAZEBO_MODEL_PATH', default=''), ':', gazebo_model_path]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    FindExecutable(name='xacro'), ' ',
                    PathJoinSubstitution([
                        FindPackageShare('ros2_dream_base_description'),
                        'urdf',
                        'diff_drive.gazebo.xacro'
                    ])
                ]),
                value_type=str
            )
        }]
    )
    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz Node (conditionally launched)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ros2_dream_base_description'),
        'config',
        'view_ros2_base.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=UnlessCondition(no_rviz)
    )

    # Include Gazebo Launch
    gazebo_launch_dir = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch'
    ])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gazebo_launch_dir,
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('ros2_dream_base_description'),
                'worlds',
                'uda_bird.world'
            ]),
            'paused': 'true',
            'use_sim_time': 'true',
            'gui': 'true',
            'record': 'false',
            'debug': 'false'
        }.items()
    )

    # Spawn Model Node
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='mybot_spawn',
        output='screen',
        arguments=[
            '-entity', 'diff_drive2',
            '-topic', 'robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos
        ]
    )

    # Define LaunchDescription and populate
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(no_rviz_arg)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)

    # # Set environment variables
    ld.add_action(set_gazebo_model_path)

    # # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_model_node)

    return ld
