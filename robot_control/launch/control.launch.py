# Copyright 2026 Mezael Docoy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import xacro


def generate_launch_description():
    description_share = get_package_share_directory(
        'robot_description'
    )
    control_share = get_package_share_directory(
        'robot_control'
    )

    xacro_file = os.path.join(
        description_share,
        'urdf',
        'robot_arm_hand.urdf.xacro'
    )
    controllers_file = os.path.join(
        control_share,
        'config',
        'controllers.yaml'
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            },
            controllers_file
        ]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    arm_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_trajectory_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    hand_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hand_position_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        arm_trajectory_controller,
        hand_position_controller
    ])
