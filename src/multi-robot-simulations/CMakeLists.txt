cmake_minimum_required(VERSION 3.8)
project(multi-robot-simulations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################
# dependencies
####################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

####################
# libraries
####################
add_library(multi_robot_library src/PoseTFPublisher.cpp)
target_include_directories(multi_robot_library PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(multi_robot_library rclcpp tf2_ros std_msgs geometry_msgs)

####################
# executables
####################
add_executable(multi_robot_simulation_main src/MultiRobotSimulationMain.cpp)
target_link_libraries(multi_robot_simulation_main multi_robot_library)
ament_target_dependencies(multi_robot_simulation_main rclcpp tf2_ros)

####################
# Instalation
####################
install(TARGETS
  multi_robot_simulation_main
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
