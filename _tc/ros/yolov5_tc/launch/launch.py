#!/usr/bin/env python

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    web_api_arg = DeclareLaunchArgument('web_api', default_value='True')

    inference_node = Node(package="yolov5_tc",
                          executable="inference_node",
                          name="inference_node",
                          output="screen",
                          emulate_tty=True,
                          parameters=[])

    web_api_node = Node(package="yolov5_tc",
                        executable="web_api",
                        name="web_api",
                        output="screen",
                        emulate_tty=True,
                        parameters=[],
                        condition=IfCondition(LaunchConfiguration('web_api')))

    return LaunchDescription([
        web_api_arg,
        inference_node,
        web_api_node
    ])
