cmake_minimum_required(VERSION 3.5)
project(ep_training)

find_package(ament_cmake REQUIRED)
find_package(ep_common REQUIRED)
find_package(ep_common_interfaces REQUIRED)
find_package(ep_training_interfaces REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(state_observer_i SHARED src/state_observer_i.cpp)
ament_target_dependencies(
    state_observer_i
    ep_common
    ep_common_interfaces
    ep_training_interfaces
    rclcpp
)

add_library(fitness_evaluator_i SHARED src/fitness_evaluator_i.cpp)
ament_target_dependencies(
    fitness_evaluator_i
    ep_common
    ep_common_interfaces
    rclcpp
    std_srvs
)

add_library(time_dependent_fitness_evaluator_i SHARED src/time_dependent_fitness_evaluator_i.cpp)
ament_target_dependencies(
    time_dependent_fitness_evaluator_i
    ep_common
    ep_common_interfaces
    rclcpp
    rosgraph_msgs
    std_srvs
)
target_link_libraries(
    time_dependent_fitness_evaluator_i
    fitness_evaluator_i
)

qt5_wrap_ui(campaign_orchestrator_gui_UI ${CMAKE_CURRENT_SOURCE_DIR}/ui/campaign_orchestrator_gui.ui)
qt5_wrap_cpp(campaign_orchestrator_gui_MOC ${CMAKE_CURRENT_SOURCE_DIR}/include/ep_training/campaign_orchestrator_gui.hpp)
add_executable(
    campaign_orchestrator_gui
    src/campaign_orchestrator_gui.cpp
    ${campaign_orchestrator_gui_UI}
    ${campaign_orchestrator_gui_MOC}
)
ament_target_dependencies(
    campaign_orchestrator_gui
    ep_training_interfaces
    rclcpp
    rclcpp_action
)
qt5_use_modules(campaign_orchestrator_gui Widgets)

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS
        state_observer_i
        fitness_evaluator_i
        time_dependent_fitness_evaluator_i
    EXPORT ep_trainingTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(
    TARGETS campaign_orchestrator_gui
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(ep_trainingTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ament_cmake
    ep_common
    ep_common_interfaces
    ep_training_interfaces
    rcl_interfaces
    rclcpp
    rclcpp_action
    rosgraph_msgs
    std_msgs
    std_srvs
)
ament_package()
