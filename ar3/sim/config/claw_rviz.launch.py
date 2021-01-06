#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

# see also xacro example at https://answers.ros.org/question/361623/ros2-robot_state_publisher-xacro-python-launch/
def generate_launch_description():
    with open("/volume/config/claw.urdf", 'r') as f:
        robot_desc_config = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc_config,
            }]),
				Node(
						package='joint_state_publisher',
						executable='joint_state_publisher',
						name='joint_state_publisher',
						output='screen',
						parameters=[{
								'robot_description': robot_desc_config,
						}]),
				Node(
						package='joint_state_publisher_gui',
						executable='joint_state_publisher_gui',
						name='joint_state_publisher_gui',
						output='screen',
				),
				Node(
						package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/volume/config/claw.rviz'],
        ),
    ])
