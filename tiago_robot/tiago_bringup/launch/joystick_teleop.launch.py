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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("tiago_bringup")

    # @TODO load the proper joy_telop config file according to tiago params
    joy_teleop_path = os.path.join(pkg_dir, "config", "joy_teleop", "joy_teleop_pal-gripper.yaml")

    declare_cmd_vel = DeclareLaunchArgument(
        "cmd_vel", default_value="input_joy/cmd_vel", description="Joystick cmd_vel topic"
    )

    declare_teleop_config = DeclareLaunchArgument(
        "teleop_config",
        default_value=joy_teleop_path,
        description="Joystick teleop configuration file",
    )

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[LaunchConfiguration("teleop_config")],
        remappings=[("cmd_vel", LaunchConfiguration("cmd_vel"))],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(pkg_dir, "config", "joy_teleop", "joy_config.yaml")],
    )

    torso_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="torso_controller",
    )

    head_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="head_controller",
    )

    gripper_incrementer_server = Node(
        package="joy_teleop",
        executable="incrementer_server",
        name="incrementer",
        namespace="gripper_controller",
    )

    ld = LaunchDescription()

    ld.add_action(declare_cmd_vel)
    ld.add_action(declare_teleop_config)

    ld.add_action(joy_teleop_node)
    ld.add_action(joy_node)

    ld.add_action(torso_incrementer_server)
    ld.add_action(head_incrementer_server)
    ld.add_action(gripper_incrementer_server)

    return ld
