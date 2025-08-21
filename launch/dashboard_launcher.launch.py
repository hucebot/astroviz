from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='astroviz',
            executable='orthogonal_viewer',
            name='orthogonal_viewer',
            output='screen'
        ),

        Node(
            package='astroviz',
            executable='robot_state_viewer',
            name='robot_state_viewer',
            output='screen'
        ),

        Node(
            package='astroviz',
            executable='motor_state_viewer',
            name='motor_state_viewer',
            output='screen'
        ),

        Node(
            package='astroviz',
            executable='imu_viewer',
            name='imu_viewer',
            output='screen'
        ),

        Node(
            package='astroviz',
            executable='lidar_viewer',
            name='lidar_viewer',
            output='screen'
        ),
    ])