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

# import os
# from pathlib import Path

# from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction

# from launch_pal.arg_utils import read_launch_argument
# from launch_pal.robot_utils import (get_arm,
#                                     get_camera_model,
#                                     get_end_effector,
#                                     get_ft_sensor,
#                                     get_laser_model,
#                                     get_robot_name,
#                                     get_wrist_model)
# from launch_param_builder import load_xacro
from launch_ros.actions import Node

# from launch.substitutions import FindExecutable
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# def declare_args(context, *args, **kwargs):

#     robot_name = read_launch_argument('robot_name', context)

#     return [get_arm(robot_name),
#             get_camera_model(robot_name),
#             get_end_effector(robot_name),
#             get_ft_sensor(robot_name),
#             get_laser_model(robot_name),
#             get_wrist_model(robot_name)]


def launch_setup(context, *args, **kwargs):

    # tiago_unrolled_urdf = 'tiago_base_unrolled.urdf'
    tiago_unrolled_urdf = "tiago_full_unrolled.urdf"

    parameters = {
        "robot_description": Command(
            [
                "cat ",
                PathJoinSubstitution(
                    [FindPackageShare("tiago_description"), "robots", tiago_unrolled_urdf]
                ),
            ]
        ).perform(context)
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[parameters],
    )

    return [rsp]


def generate_launch_description():

    ld = LaunchDescription()

    # Declare arguments
    # we use OpaqueFunction so the callbacks have access to the context

    # ld.add_action(get_robot_name('tiago'))
    # ld.add_action(OpaqueFunction(function=declare_args))

    # Execute robot_state_publisher node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
