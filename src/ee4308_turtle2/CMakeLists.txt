cmake_minimum_required(VERSION 3.8)
project(ee4308_turtle2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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
)

######## MAP_LOADER ########
add_library(map_loader_component SHARED src/map_loader.cpp)
target_include_directories(map_loader_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_loader_component ${DEPENDENCIES})
target_compile_features(map_loader_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
rclcpp_components_register_node(
    map_loader_component
    PLUGIN "ee4308::turtle2::MapLoader"
    EXECUTABLE map_loader
)
ament_export_targets(export_map_loader_component)
install(TARGETS map_loader_component
        EXPORT export_map_loader_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

######## BEHAVIOR ########
add_library(behavior_component SHARED src/behavior.cpp)
target_include_directories(behavior_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(behavior_component ${DEPENDENCIES})
target_compile_features(behavior_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
rclcpp_components_register_node(
    behavior_component
    PLUGIN "ee4308::turtle2::Behavior"
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
    PLUGIN "ee4308::turtle2::Controller"
    EXECUTABLE controller
)
ament_export_targets(export_controller_component)
install(TARGETS controller_component
        EXPORT export_controller_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

######## PLANNER ########
add_library(planner_component SHARED src/planner.cpp)
# target_include_directories(planner_component PUBLIC include)
target_include_directories(planner_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planner_component ${DEPENDENCIES})
target_compile_features(planner_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
rclcpp_components_register_node(
    planner_component
    PLUGIN "ee4308::turtle2::Planner"
    EXECUTABLE planner
)
ament_export_targets(export_planner_component)
install(TARGETS planner_component
        EXPORT export_planner_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_package()