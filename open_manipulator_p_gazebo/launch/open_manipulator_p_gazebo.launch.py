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
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


#(ref.) https://answers.ros.org/question/373824/ros2-launch-file-arguments-subsititutions-xacro-and-node-parameters/
#(ref.) https://zmk5.github.io/general/demo/2019/07/15/ros2-spawning-entity.html

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    with_gripper = LaunchConfiguration('with_gripper', default='False')
    
    world_file_name = 'empty_world.model'
    world = os.path.join(get_package_share_directory('open_manipulator_p_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('open_manipulator_p_gazebo'), 'launch')

    urdf_file_name = 'open_manipulator_p_robot.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(get_package_share_directory('open_manipulator_p_gazebo'), 'urdf',
      urdf_file_name)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/open_manipulator_p_spawner.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items()
        #     launch_arguments={'with_gripper': with_gripper}.items()
        #     arguments=[robot_description],
        # ),
    ])
