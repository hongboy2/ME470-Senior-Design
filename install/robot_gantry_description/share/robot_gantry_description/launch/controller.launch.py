# Author: George Zhai
# Date: February 24, 2023
# Description: Launch the controller of gantry robot in Gazebo

import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen'
            args=
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
