cmake_minimum_required(VERSION 3.5)
project(ep_gantry_velocity_control_demo)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    ament_cmake
)
ament_package()
