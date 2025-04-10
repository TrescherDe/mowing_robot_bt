import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Task Planner Node (C++)
        Node(
            package='mowing_robot_bt',
            executable='task_planner',
            name='task_planner',
            output='screen'
        ),

        # Static TF Publisher (Python)
        Node(
            package='mowing_robot_bt',
            executable='static_tf2_publisher.py',
            name='static_tf2_publisher',
            output='screen'
        ),

        # Neural Net (Python)
        Node(
            package='mowing_robot_bt',
            executable='NeuralNet.py',
            name='NeuralNet',
            output='screen'
        ),

        # IPM Node (Python)
        Node(
            package='mowing_robot_bt',
            executable='IPM.py',
            name='IPM',
            output='screen'
        )
    ])
