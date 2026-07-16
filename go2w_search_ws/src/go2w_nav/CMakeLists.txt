cmake_minimum_required(VERSION 3.5)
project(go2w_nav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mid360_nav_bridge_cpp src/mid360_nav_bridge.cpp)
ament_target_dependencies(
  mid360_nav_bridge_cpp rclcpp sensor_msgs)
target_compile_features(mid360_nav_bridge_cpp PUBLIC cxx_std_17)
target_compile_options(mid360_nav_bridge_cpp PRIVATE -O3)

install(TARGETS mid360_nav_bridge_cpp DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY behavior_trees/ DESTINATION share/${PROJECT_NAME}/behavior_trees)

ament_package()
