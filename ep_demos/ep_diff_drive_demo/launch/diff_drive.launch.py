# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    controller_parameters_path = LaunchConfiguration("controller_parameters_path")
    declare_controller_parameters_path_cmd = DeclareLaunchArgument(
        "controller_parameters_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("ep_diff_drive_demo"),
            "config",
            "sample.yaml",
        ]),
        description="The path to the YAML file of the parameters for the controller to use"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            ])
        ])
    )

    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ep_diff_drive_demo"), "urdf", "diff_drive.urdf.xacro"]),
            " controller_parameters_path:=",
            controller_parameters_path,
        ])
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "cartpole",
        ],
        output="both",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start", "joint_state_broadcaster"],
        output="both",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start", "diff_drive_base_controller"],
        output="both",
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
