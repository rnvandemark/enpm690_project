cmake_minimum_required(VERSION 3.5)
project(ep_common)

find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(node_id SHARED src/node_id.cpp)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS node_id
    EXPORT node_idTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(node_idTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ament_cmake
)
ament_package()
