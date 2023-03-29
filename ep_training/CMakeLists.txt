cmake_minimum_required(VERSION 3.5)
project(ep_training)

find_package(ament_cmake REQUIRED)
find_package(ep_common REQUIRED)
find_package(ep_common_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(fitness_evaluator_i SHARED src/fitness_evaluator_i.cpp)
ament_target_dependencies(
    fitness_evaluator_i
    ep_common
    ep_common_interfaces
    rclcpp
    std_srvs
)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS fitness_evaluator_i
    EXPORT fitness_evaluator_iTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(fitness_evaluator_iTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ament_cmake
    ep_common
    ep_common_interfaces
    rclcpp
    std_srvs
)
ament_package()
