cmake_minimum_required(VERSION 3.8)
project(ee4308_drone)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

set (DEPENDENCIES
  rclcpp
  rclcpp_components
  rcutils
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  Eigen3
)

######### BEHAVIOR ########
add_library(behavior_component SHARED src/behavior.cpp)
target_include_directories(behavior_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(behavior_component ${DEPENDENCIES})
target_compile_features(behavior_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
rclcpp_components_register_node(
    behavior_component
    PLUGIN "ee4308::drone::Behavior"
    EXECUTABLE behavior
)
ament_export_targets(export_behavior_component)
install(TARGETS behavior_component
        EXPORT export_behavior_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

######## CONTROLLER ########
add_library(controller_component SHARED src/controller.cpp)
target_include_directories(controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(controller_component ${DEPENDENCIES})
target_compile_features(controller_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
rclcpp_components_register_node(
    controller_component
    PLUGIN "ee4308::drone::Controller"
    EXECUTABLE controller
)
ament_export_targets(export_controller_component)
install(TARGETS controller_component
        EXPORT export_controller_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

######## ESTIMATOR ########
add_library(estimator_component SHARED src/estimator.cpp)
target_include_directories(estimator_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(estimator_component ${DEPENDENCIES})
target_compile_features(estimator_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
rclcpp_components_register_node(
    estimator_component
    PLUGIN "ee4308::drone::Estimator"
    EXECUTABLE estimator
)
ament_export_targets(export_estimator_component)
install(TARGETS estimator_component
        EXPORT export_estimator_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

######## PLANNER ########
add_library(planner_component SHARED src/planner.cpp)
target_include_directories(planner_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planner_component ${DEPENDENCIES})
target_compile_features(planner_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
rclcpp_components_register_node(
    planner_component
    PLUGIN "ee4308::drone::Planner"
    EXECUTABLE planner
)
ament_export_targets(export_planner_component)
install(TARGETS planner_component
        EXPORT export_planner_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_package()
