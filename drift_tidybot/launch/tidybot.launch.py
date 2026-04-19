import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Gazebo, spawn the robot, load controllers, and start nodes."""
    pkg = get_package_share_directory('drift_tidybot')
    urdf_path = os.path.join(pkg, 'urdf', 'robot.urdf')
    rviz_config_path = os.path.join(pkg, 'rviz', 'config.rviz')
    world_path = os.path.join(pkg, 'worlds', 'tidybot_home.world')

    with open(urdf_path, 'r', encoding='utf-8') as file:
        robot_desc = file.read()

    robot_desc = robot_desc.replace(
        '<?xml version="1.0" encoding="utf-8"?>',
        '<?xml version="1.0"?>',
    )
    robot_desc = robot_desc.replace('$(find drift_tidybot)', pkg)

    set_gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg, '..'),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'pause': 'false',
            'verbose': 'false',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tidybot',
            '-x', '0.0', '-y', '0', '-z', '0.5',
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    navigate_node = Node(
        package='drift_tidybot',
        executable='navigate.py',
        name='navigator',
        output='screen',
    )

    camera_node = Node(
        package='drift_tidybot',
        executable='camera.py',
        name='vision_test',
        output='screen',
    )

    spawn_delayed = TimerAction(period=3.0, actions=[spawn_robot])
    navigate_delayed = TimerAction(period=8.0, actions=[navigate_node])
    camera_delayed = TimerAction(period=9.0, actions=[camera_node])

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_delayed,
        rviz_node,
        joint_state_broadcaster,
        arm_controller,
        navigate_delayed,
        camera_delayed,
    ])
