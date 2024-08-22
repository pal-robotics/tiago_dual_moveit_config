# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_dual_description.launch_arguments import TiagoDualArgs
from tiago_dual_description.tiago_dual_launch_utils import get_tiago_dual_hw_suffix

from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoDualArgs.base_type
    arm_type_right: DeclareLaunchArgument = TiagoDualArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoDualArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoDualArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoDualArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoDualArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoDualArgs.ft_sensor_left
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    use_sensor_manager: DeclareLaunchArgument = CommonArgs.use_sensor_manager


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(function=start_move_group))
    return


def start_move_group(context, *args, **kwargs):

    base_type = read_launch_argument("base_type", context)
    arm_type_right = read_launch_argument('arm_type_right', context)
    arm_type_left = read_launch_argument('arm_type_left', context)
    end_effector_right = read_launch_argument('end_effector_right', context)
    end_effector_left = read_launch_argument('end_effector_left', context)
    ft_sensor_right = read_launch_argument('ft_sensor_right', context)
    ft_sensor_left = read_launch_argument('ft_sensor_left', context)
    use_sensor_manager = read_launch_argument('use_sensor_manager', context)

    hw_suffix = get_tiago_dual_hw_suffix(
        arm_right=arm_type_right,
        arm_left=arm_type_left,
        end_effector_right=end_effector_right,
        end_effector_left=end_effector_left)

    srdf_file_path = Path(
        os.path.join(
            get_package_share_directory("tiago_dual_moveit_config"),
            "config", "srdf",
            "tiago_dual.srdf.xacro",
        )
    )

    srdf_input_args = {
        "arm_type_right": arm_type_right,
        "arm_type_left": arm_type_left,
        "end_effector_right": end_effector_right,
        "end_effector_left": end_effector_left,
        "ft_sensor_right": ft_sensor_right,
        "ft_sensor_left": ft_sensor_left,
        "base_type": base_type,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        f'config/controllers/controllers{hw_suffix}.yaml')

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder('tiago_dual')
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
    )

    if use_sensor_manager:
        # moveit_sensors path
        moveit_sensors_path = 'config/sensors_3d.yaml'
        moveit_config.sensors_3d(moveit_sensors_path)

    moveit_config.to_moveit_configs()

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            moveit_config.to_dict(),
            {'publish_robot_description_semantic': True}
        ],
    )

    return [run_move_group_node]
