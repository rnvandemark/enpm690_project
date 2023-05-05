cmake_minimum_required(VERSION 3.5)
project(ep_gantry_velocity_control_demo)

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ep_common REQUIRED)
find_package(ep_training REQUIRED)
find_package(ep_training_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

add_library(velocity_pid_controller SHARED src/velocity_pid_controller.cpp)
ament_target_dependencies(
    velocity_pid_controller
    ep_common
    pluginlib
)
pluginlib_export_plugin_description_file(
    controller_interface
    ep_gantry_velocity_control_demo_plugin.xml
)

add_executable(pid_solution_genetic_algorithm_controller src/pid_solution_genetic_algorithm_controller.cpp)
ament_target_dependencies(
    pid_solution_genetic_algorithm_controller
    controller_manager_msgs
    ep_training
    ep_training_interfaces
    rclcpp_action
)

add_executable(velocity_pid_state_observer src/velocity_pid_state_observer.cpp)
ament_target_dependencies(
    velocity_pid_state_observer
    ep_training
    sensor_msgs
)

add_executable(velocity_pid_fitness_evaluator src/velocity_pid_fitness_evaluator.cpp)
ament_target_dependencies(
    velocity_pid_fitness_evaluator
    ep_training
    sensor_msgs
)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS velocity_pid_controller
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(
    TARGETS
        pid_solution_genetic_algorithm_controller
        velocity_pid_state_observer
        velocity_pid_fitness_evaluator
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)
install(
    DIRECTORY launch urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
    ament_cmake
    controller_manager_msgs
    ep_common
    ep_training
    pluginlib
)
ament_package()
