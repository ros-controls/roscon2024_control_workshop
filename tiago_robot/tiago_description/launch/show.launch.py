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

from typing import List

from launch import LaunchDescription, SomeSubstitutionsType
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def include_launch_py_description(
    pkg_name: SomeSubstitutionsType, paths: List[SomeSubstitutionsType], **kwargs
) -> str:
    """
    Return IncludeLaunchDescription for the file inside pkg at paths.

    Example:
    -------
        include_launch_py_description('my_pkg', ['launch', 'my_file.launch.py'])
        returns file IncludeLaunchDescription from PATH_TO_MY_PKG_SHARE/launch/my_file.launch.py

    """
    pkg_dir = FindPackageShare(pkg_name)
    full_path = PathJoinSubstitution([pkg_dir] + paths)

    return IncludeLaunchDescription(PythonLaunchDescriptionSource(full_path), **kwargs)


def generate_launch_description():

    robot_state_publisher = include_launch_py_description(
        "tiago_description", ["launch", "robot_state_publisher.launch.py"]
    )

    start_joint_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("tiago_description"), "config", "model_and_tf.rviz"]
            ),
        ],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(start_joint_pub_gui)
    ld.add_action(start_rviz_cmd)

    return ld
