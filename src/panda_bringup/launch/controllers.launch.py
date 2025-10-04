# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare use_sim_time argument
    decleare_use_sim_time   = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock if true")
    declare_namespace       = DeclareLaunchArgument("namespace", default_value="", description="The namespace of the robot")
    declare_ctrl_file_name  = DeclareLaunchArgument("controllers_file_name", default_value="ros2_controllers.yaml", description="The filename for the robot's controller")
    declare_use_hand_ctrl   = DeclareLaunchArgument("use_hand_ctrl", default_value="false")

    use_sim_time                = LaunchConfiguration("use_sim_time")
    namespace                   = LaunchConfiguration('namespace')
    controllers_file_name       = LaunchConfiguration('controllers_file_name')

    ros2_controllers_path = PathJoinSubstitution([
        get_package_share_directory("panda_bringup"),
        "config",
        controllers_file_name,
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            ros2_controllers_path,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["panda_arm_controller"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=[
            "panda_hand_controller", 
        ],
        condition=IfCondition(LaunchConfiguration("use_hand_ctrl"))
    )

    joint_states_topic_relay = Node(
        package="topic_tools",
        namespace=namespace,
        executable="relay",
        arguments=[
            [TextSubstitution(text="/"), namespace, TextSubstitution(text="/joint_states")], "/joint_states" 
        ],
    )

    # delayed_spawners = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=ros2_control_node,
    #         on_start=[
    #             joint_state_broadcaster_spawner,
    #             panda_arm_controller_spawner,
    #             panda_hand_controller_spawner,
    #         ],
    #     )
    # )

    return LaunchDescription(
        [
            decleare_use_sim_time,
            declare_use_hand_ctrl,
            declare_namespace,
            declare_ctrl_file_name,

            joint_states_topic_relay,
            ros2_control_node,
            # delayed_spawners
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner,
        ]
    )