#!/usr/bin/env python

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    test_arg = DeclareLaunchArgument('test', default_value='False')

    always_run_node = Node(package="yolov5_tc",
                           executable="inference_node",
                           name="inference_node",
                           output="screen",
                           emulate_tty=True,
                           parameters=[])

    conditional_node = Node(package="yolov5_tc",
                            executable="image_publisher",
                            name="image_publisher",
                            output="screen",
                            emulate_tty=True,
                            parameters=[],
                            condition=IfCondition(LaunchConfiguration('test')))

    return LaunchDescription([
        test_arg,
        always_run_node,
        conditional_node
    ])
