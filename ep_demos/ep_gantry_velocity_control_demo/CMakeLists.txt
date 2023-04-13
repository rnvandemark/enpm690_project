cmake_minimum_required(VERSION 3.5)
project(ep_gantry_velocity_control_demo)

find_package(ament_cmake REQUIRED)
find_package(ep_common REQUIRED)
find_package(pluginlib REQUIRED)

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

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS velocity_pid_controller
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
    ep_common
    pluginlib
)
ament_package()
