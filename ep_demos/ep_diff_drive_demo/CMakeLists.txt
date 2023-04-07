cmake_minimum_required(VERSION 3.5)
project(ep_diff_drive_demo)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(ep_diff_drive_demo)
ament_export_dependencies(
    ament_cmake
)
ament_package()
