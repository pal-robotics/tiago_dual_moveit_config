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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from tiago_dual_description.tiago_dual_launch_utils import get_tiago_dual_hw_suffix
from launch_pal.arg_utils import LaunchArgumentsBase, launch_arg_factory
from dataclasses import dataclass


@dataclass(frozen=True)
class ArgumentDeclaration(LaunchArgumentsBase):
    use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation time')


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    robot_name = "tiago_dual"
    has_robot_config = True
    custom_args = ArgumentDeclaration()
    launch_args = launch_arg_factory(custom_args,
                                     has_robot_config=has_robot_config, robot_name=robot_name)

    launch_args.add_to_launch_description(ld)

    declare_actions(ld, launch_args)

    return ld


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArgumentsBase):

    launch_description.add_action(OpaqueFunction(function=start_rviz))
    return


def start_rviz(context, *args, **kwargs):

    arm_type_right = read_launch_argument('arm_type_right', context)
    arm_type_left = read_launch_argument('arm_type_left', context)
    end_effector_right = read_launch_argument('end_effector_right', context)
    end_effector_left = read_launch_argument('end_effector_left', context)
    ft_sensor_right = read_launch_argument('ft_sensor_right', context)
    ft_sensor_left = read_launch_argument('ft_sensor_left', context)
    wrist_model_right = read_launch_argument('wrist_model_right', context)
    wrist_model_left = read_launch_argument('wrist_model_left', context)
    camera_model = read_launch_argument('camera_model', context)
    laser_model = read_launch_argument('laser_model', context)
    base_type = read_launch_argument('base_type', context)

    robot_description_path = os.path.join(
        get_package_share_directory('tiago_dual_description'), 'robots', 'tiago_dual.urdf.xacro')

    mappings = {
        'arm_type_right': arm_type_right,
        'arm_type_left': arm_type_left,
        'end_effector_right': end_effector_right,
        'end_effector_left': end_effector_left,
        'ft_sensor_right': ft_sensor_right,
        'ft_sensor_left': ft_sensor_left,
        'wrist_model_right': wrist_model_right,
        'wrist_model_left': wrist_model_left,
        'camera_model': camera_model,
        'laser_model': laser_model,
        'base_type': base_type,
    }

    hw_suffix = get_tiago_dual_hw_suffix(
        arm_right=arm_type_right,
        arm_left=arm_type_left,
        end_effector_right=end_effector_right,
        end_effector_left=end_effector_left,
        ft_sensor_right=ft_sensor_right,
        ft_sensor_left=ft_sensor_left)

    robot_description_semantic = (f'config/srdf/tiago_dual_{hw_suffix}.srdf')

    if base_type == "omni_base":
        robot_description_semantic = (
            f'config/srdf/tiago_dual_omni_{hw_suffix}.srdf')

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        f'config/controllers/controllers_{hw_suffix}.yaml')

    moveit_config = (
        MoveItConfigsBuilder('tiago_dual')
        .robot_description(file_path=robot_description_path, mappings=mappings)
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'tiago_dual_moveit_config'), 'config', 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            {},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return [rviz_node]
