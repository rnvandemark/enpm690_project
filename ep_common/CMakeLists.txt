cmake_minimum_required(VERSION 3.5)
project(ep_common)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ep_common_interfaces REQUIRED)

include_directories(include)

add_library(node_id SHARED src/node_id.cpp)

add_library(command_interface_controller_i SHARED src/command_interface_controller_i.cpp)
ament_target_dependencies(
    command_interface_controller_i
    controller_interface
    ep_common_interfaces
)
target_link_libraries(command_interface_controller_i node_id)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS
        node_id
        command_interface_controller_i
    EXPORT ep_commonTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(ep_commonTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ament_cmake
    controller_interface
    ep_common_interfaces
)
ament_package()
