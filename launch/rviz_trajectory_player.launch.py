from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_file_arg =  DeclareLaunchArgument('robot_file')
    robot_file = LaunchConfiguration('robot_file')

    trajectory_file_arg =  DeclareLaunchArgument('trajectory_file')
    trajectory_file = LaunchConfiguration('trajectory_file')

    robot_xml = Command(["xacro ", robot_file])

    base_path = get_package_share_directory("rviz_trajectory_player")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([base_path, 'config', 'viz.rviz']),
            '-f',
            'world'
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': ParameterValue(robot_xml, value_type=str),
                     'use_sim_time':True}],
    )

    base_to_world_tf = Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        name='base_to_world', 
        output='log', 
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',  '/base_link', '/world']
    )

    trajectory_republisher = Node(
        package='rviz_trajectory_player',
        executable='trajectory_republisher',
        name='visualizer_trajectory_republisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_xml, value_type=str),
             'trajectory_file': ParameterValue(trajectory_file, value_type=str)}
        ]
    )

    return LaunchDescription([rviz, robot_state_publisher, base_to_world_tf, trajectory_republisher])
