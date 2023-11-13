from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robocol_arm_moveit2',
            executable='pilz_planning',
            name='planner'
        ),
        Node(
            package='robocol_arm_moveit2',
            executable='traj_extractor',
            name='extractor'
        ),
        Node(
            package='gcode_executor',
            executable='gcode_reader',
            name='gcoder',
        ),
        Node(
            package='gcode_executor',
            executable='gui',
            name='interface',
        )
    ])