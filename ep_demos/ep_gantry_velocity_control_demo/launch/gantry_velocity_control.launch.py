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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    controller_parameters_path = LaunchConfiguration("controller_parameters_path")
    declare_controller_parameters_path_cmd = DeclareLaunchArgument(
        "controller_parameters_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("ep_gantry_velocity_control_demo"),
            "config",
            "sample.yaml",
        ]),
        description="The path to the YAML file of the parameters for the controller to use",
    )

    motion_profile_time_accelerating = LaunchConfiguration("motion_profile_time_accelerating")
    declare_motion_profile_time_accelerating_cmd = DeclareLaunchArgument(
        "motion_profile_time_accelerating",
        default_value="",
        description="The amount of time to spend accelerating and decelerating each in the motion.",
    )

    motion_profile_time_constant_velocity = LaunchConfiguration("motion_profile_time_constant_velocity")
    declare_motion_profile_time_constant_velocity_cmd = DeclareLaunchArgument(
        "motion_profile_time_constant_velocity",
        default_value="",
        description="The amount of time to spend at constant velocity in the motion.",
    )

    motion_profile_acceleration = LaunchConfiguration("motion_profile_acceleration")
    declare_motion_profile_acceleration_cmd = DeclareLaunchArgument(
        "motion_profile_acceleration",
        default_value="",
        description="The acceleration during the acceleration and deceleration sequences in the motion.",
    )

    target_displacement = LaunchConfiguration("target_displacement")
    declare_target_displacement_cmd = DeclareLaunchArgument(
        "target_displacement",
        default_value=PythonExpression([
            motion_profile_acceleration,
            "*",
            motion_profile_time_accelerating,
            "*(",
            motion_profile_time_accelerating,
            "+",
            motion_profile_time_constant_velocity,
            ")",
        ])
    )
    log_target_displacement = LogInfo(msg=[
        "With acc = ", motion_profile_acceleration, " m/s^2, t_a = ", motion_profile_time_accelerating,
        " s, t_cv = ", motion_profile_time_constant_velocity, " s -> dx = ", target_displacement, " m",
    ])

    target_displacement_is_positive = LaunchConfiguration("target_displacement_is_positive")
    declare_target_displacement_is_positive_cmd = DeclareLaunchArgument(
        "target_displacement_is_positive",
        default_value=PythonExpression([
            "True if (",
            target_displacement,
            " >= 0.0) else False",
        ])
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
            PathJoinSubstitution([FindPackageShare("ep_gantry_velocity_control_demo"), "urdf", "gantry_velocity_control.urdf.xacro"]),
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

    node_campaign_orchestrator_gui = Node(
        package="ep_training",
        executable="campaign_orchestrator_gui",
        output="both",
    )

    node_genetic_algorithm_controller = Node(
        package="ep_gantry_velocity_control_demo",
        executable="pid_solution_genetic_algorithm_controller",
        output="both",
    )

    node_state_observer = Node(
        package="ep_gantry_velocity_control_demo",
        executable="velocity_pid_state_observer",
        parameters=[{
            "use_sim_time": True,
            "target_position": target_displacement,
            "target_displacement_is_positive": target_displacement_is_positive,
        }],
        output="both",
    )

    node_fitness_evaluator = Node(
        package="ep_gantry_velocity_control_demo",
        executable="velocity_pid_fitness_evaluator",
        parameters=[{
            "profile.acceleration": motion_profile_acceleration,
            "profile.t_accelerating": motion_profile_time_accelerating,
            "profile.t_constant_velocity": motion_profile_time_constant_velocity,
        }],
        output="both",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start", "joint_state_broadcaster"],
        output="both",
    )

    load_imu_sensor_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "imu_sensor_broadcaster"],
        output="both"
    )

    load_velocity_pid_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "command_interface_controller_0"],
        output="both",
    )

    set_velocity_pid_controller_acceleration_param = ExecuteProcess(
        cmd=["ros2", "param", "set", "command_interface_controller_0", "profile.acceleration", motion_profile_acceleration],
        output="both",
    )

    set_velocity_pid_controller_t_accelerating_param = ExecuteProcess(
        cmd=["ros2", "param", "set", "command_interface_controller_0", "profile.t_accelerating", motion_profile_time_accelerating],
        output="both",
    )

    set_velocity_pid_controller_t_constant_velocity_param = ExecuteProcess(
        cmd=["ros2", "param", "set", "command_interface_controller_0", "profile.t_constant_velocity", motion_profile_time_constant_velocity],
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
                on_exit=[load_imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_imu_sensor_broadcaster,
                on_exit=[load_velocity_pid_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_pid_controller,
                on_exit=[set_velocity_pid_controller_acceleration_param],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_pid_controller,
                on_exit=[set_velocity_pid_controller_t_accelerating_param],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_pid_controller,
                on_exit=[set_velocity_pid_controller_t_constant_velocity_param],
            )
        ),
        declare_controller_parameters_path_cmd,
        declare_motion_profile_time_accelerating_cmd,
        declare_motion_profile_time_constant_velocity_cmd,
        declare_motion_profile_acceleration_cmd,
        declare_target_displacement_cmd,
        declare_target_displacement_is_positive_cmd,
        log_target_displacement,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        node_campaign_orchestrator_gui,
        node_genetic_algorithm_controller,
        node_state_observer,
        node_fitness_evaluator,
    ])
