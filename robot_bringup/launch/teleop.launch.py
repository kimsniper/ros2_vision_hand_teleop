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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    arm_control_mode = LaunchConfiguration(
        'arm_control_mode'
    )

    state_estimator_node = Node(
        package="robot_state_estimation",
        executable="state_estimator_node",
        output="screen"
    )

    cartesian_delta_controller = Node(
        package='robot_control',
        executable='cartesian_delta_control_node',
        output='screen',
        parameters=[{
            'trajectory_duration': 0.08
        }],
        condition=IfCondition(
            PythonExpression([
                "'", arm_control_mode,
                "' == 'cartesian_delta'"
            ])
        )
    )

    cartesian_controller = Node(
        package='robot_control',
        executable='cartesian_control_node',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", arm_control_mode,
                "' == 'cartesian'"
            ])
        )
    )

    arm_cartesian_delta_teleop = Node(
        package='robot_teleop',
        executable='arm_cartesian_delta_teleop_node',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", arm_control_mode,
                "' == 'cartesian_delta'"
            ])
        )
    )

    arm_cartesian_teleop = Node(
        package='robot_teleop',
        executable='arm_cartesian_teleop_node',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", arm_control_mode,
                "' == 'cartesian'"
            ])
        )
    )

    hand_teleop = Node(
        package='robot_teleop',
        executable='hand_teleop_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_control_mode',
            default_value='cartesian',
            choices=[
                'cartesian',
                'cartesian_delta'
            ],
            description=(
                'Select primary cartesian or secondary '
                'cartesian_delta arm control'
            )
        ),
        state_estimator_node,
        cartesian_delta_controller,
        cartesian_controller,
        arm_cartesian_delta_teleop,
        arm_cartesian_teleop,
        hand_teleop
    ])
