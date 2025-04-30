import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('diff_drive_robot')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(urdf_file).read()}]
        ),

        # Sensor Node
        Node(
            package='diff_drive_robot',
            executable='sensor_node',
            name='sensor_node',
            output='screen'
        ),

        # Motor Control Node
        Node(
            package='diff_drive_robot',
            executable='motor_control',
            name='motor_control',
            output='screen'
        ),

        # Obstacle Avoidance Node
        Node(
            package='diff_drive_robot',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'robot.rviz')]
        )
    ])
