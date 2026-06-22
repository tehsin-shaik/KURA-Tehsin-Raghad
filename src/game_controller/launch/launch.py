# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package ='game_controller',
            executable='game_controller_node',
            name='game_controller',
            output='screen',
            parameters=[
                {
                    # GC broadcast port (rule-fixed, do not change)
                    "port": 3838,

                    # Disable IP whitelist for the migration. Re-enable for the
                    # competition by setting True and listing the GC laptop IP(s)
                    # in ip_white_list below.
                    "enable_ip_white_list": True,

                    # Add the new GC laptop's IP(s) here before re-enabling.
                    "ip_white_list": [
                        "192.168.0.126",
                        "192.168.0.206",
                        "192.168.41.2",
                    ],
                }
            ]
        ),
    ])
