#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Ashe Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    with_gripper = LaunchConfiguration('with_gripper', default='False')
    param_dir = LaunchConfiguration(
      'param_dir',
      default=os.path.join(
          get_package_share_directory('open_manipulator_p_gazebo'),
          'param',
          'arm_controller.yaml'))
          

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=["joint1_position", "joint2_position", "joint3_position", 
                    "joint4_position", "joint5_position", "joint6_position"],
            output='screen'),
    ])


