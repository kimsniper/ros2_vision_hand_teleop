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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
import xacro
import yaml


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    description_share = get_package_share_directory(
        'robot_description'
    )
    control_share = get_package_share_directory(
        'robot_control'
    )
    moveit_share = get_package_share_directory(
        'robot_moveit'
    )

    xacro_file = os.path.join(
        description_share,
        'urdf',
        'robot_arm_hand.urdf.xacro'
    )
    srdf_file = os.path.join(
        moveit_share,
        'config',
        'robot.srdf'
    )
    kinematics_file = os.path.join(
        moveit_share,
        'config',
        'kinematics.yaml'
    )
    planning_file = os.path.join(
        moveit_share,
        'config',
        'ompl_planning.yaml'
    )
    controllers_file = os.path.join(
        moveit_share,
        'config',
        'controllers.yaml'
    )

    robot_description = {
        'robot_description': xacro.process_file(
            xacro_file
        ).toxml()
    }

    with open(srdf_file, 'r', encoding='utf-8') as stream:
        robot_description_semantic = {
            'robot_description_semantic': stream.read()
        }

    robot_description_kinematics = load_yaml(
        kinematics_file
    )
    planning_pipeline_config = load_yaml(
        planning_file
    )
    planning_pipeline_config.update({
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters':
            'default_planner_request_adapters/'
            'AddTimeOptimalParameterization '
            'default_planner_request_adapters/'
            'ResolveConstraintFrames '
            'default_planner_request_adapters/'
            'FixWorkspaceBounds '
            'default_planner_request_adapters/'
            'FixStartStateBounds '
            'default_planner_request_adapters/'
            'FixStartStateCollision '
            'default_planner_request_adapters/'
            'FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1
    })
    moveit_controllers = load_yaml(
        controllers_file
    )

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/'
            'MoveItSimpleControllerManager',
        'moveit_simple_controller_manager':
            moveit_controllers
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True
    }

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                control_share,
                'launch',
                'control.launch.py'
            )
        )
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl'
            },
            {
                'ompl': planning_pipeline_config
            },
            trajectory_execution,
            planning_scene_monitor,
            {
                'planning_scene_monitor_options': {
                    'name': 'planning_scene_monitor',
                    'robot_description': 'robot_description',
                    'joint_state_topic': '/joint_states',
                    'attached_collision_object_topic':
                        '/move_group/planning_scene_monitor',
                    'publish_planning_scene_topic':
                        '/move_group/publish_planning_scene',
                    'monitored_planning_scene_topic':
                        '/move_group/monitored_planning_scene',
                    'wait_for_initial_state_timeout': 10.0
                }
            },
            {
                'capabilities': '',
                'disable_capabilities': ''
            }
        ]
    )

    return LaunchDescription([
        control_launch,
        move_group
    ])
