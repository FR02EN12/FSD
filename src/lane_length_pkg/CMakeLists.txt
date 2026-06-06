cmake_minimum_required(VERSION 3.8)
project(lane_length_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(rcutils REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR lane_length_pkg/${PROJECT_NAME}
)

function(install_python_node executable module)
  set(wrapper_dir "${CMAKE_CURRENT_BINARY_DIR}/python_nodes")
  set(wrapper_path "${wrapper_dir}/${executable}")
  file(MAKE_DIRECTORY "${wrapper_dir}")
  file(WRITE "${wrapper_path}"
"#!/usr/bin/env python3
from ${PROJECT_NAME}.${module} import main

if __name__ == '__main__':
    main()
")
  file(CHMOD "${wrapper_path}"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_WRITE GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE)
  install(PROGRAMS "${wrapper_path}" DESTINATION lib/${PROJECT_NAME})
endfunction()

# Existing lane_length_pkg Python entry points.
install_python_node(detection_node lane_detection_node)
install_python_node(integration_node lane_integration_node)
install_python_node(control_node lane_follow_control_node)
install_python_node(decision_node lane_decision_node)
install_python_node(viewer_node lane_viewer_node)
install_python_node(lane_detection_node lane_detection_node)
install_python_node(lane_integration_node lane_integration_node)
install_python_node(lane_decision_node lane_decision_node)
install_python_node(lane_follow_control_node lane_follow_control_node)
install_python_node(lane_viewer_node lane_viewer_node)
install_python_node(scene_fusion_node scene_fusion_node)
install_python_node(drive_mode_node drive_mode_node)
install_python_node(behavior_plan_node behavior_plan_node)
install_python_node(reverse_astar_planner_node reverse_astar_planner_node)
install_python_node(path_track_node path_track_node)
install_python_node(lane_follow_eval lane_follow_eval_node)
install_python_node(cmd_mux_node cmd_mux_node)
install_python_node(safety_filter_node safety_filter_node)

# src workspace Python nodes copied into this package under their original names.
install_python_node(camera_driver camera_driver)
install_python_node(object_detect object_detect)
install_python_node(led_signal led_signal)
install_python_node(space_memory space_memory)
install_python_node(situation_fusion situation_fusion)
install_python_node(deadlock deadlock)
install_python_node(cooperation cooperation)
install_python_node(topic_watchdog topic_watchdog)
install_python_node(led_behavior led_behavior)
install_python_node(led_io led_io)

# perception_cpp
add_executable(localization_slam src/perception_cpp/localization_slam.cpp)
ament_target_dependencies(localization_slam
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)

add_library(turtlebot3_control_lib
  src/control_cpp/devices/motor_power.cpp
  src/control_cpp/devices/sound.cpp
  src/control_cpp/devices/reset.cpp
  src/control_cpp/diff_drive_controller.cpp
  src/control_cpp/dynamixel_sdk_wrapper.cpp
  src/control_cpp/odometry.cpp
  src/control_cpp/turtlebot3.cpp
  src/control_cpp/sensors/battery_state.cpp
  src/control_cpp/sensors/imu.cpp
  src/control_cpp/sensors/joint_state.cpp
  src/control_cpp/sensors/sensor_state.cpp
)
target_include_directories(turtlebot3_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${dynamixel_sdk_INCLUDE_DIRS}
)
target_link_libraries(turtlebot3_control_lib
  ${dynamixel_sdk_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${turtlebot3_msgs_TARGETS}
  message_filters::message_filters
  rclcpp::rclcpp
  rcutils::rcutils
  tf2::tf2
  tf2_ros::tf2_ros
)

add_executable(turtlebot3_ros src/control_cpp/node_main.cpp)
target_link_libraries(turtlebot3_ros turtlebot3_control_lib)

# ld08_driver
file(GLOB LD08_DRIVER_SRC CONFIGURE_DEPENDS drivers/ld08_driver/src/*.cpp)
add_executable(ld08_driver ${LD08_DRIVER_SRC})
target_include_directories(ld08_driver PRIVATE drivers/ld08_driver/include)
ament_target_dependencies(ld08_driver rclcpp sensor_msgs std_msgs)
target_link_libraries(ld08_driver Boost::system)
if(UNIX AND NOT APPLE)
  target_link_libraries(ld08_driver pthread udev)
endif()

# coin_d4_driver
add_library(coin_d4_driver_lib SHARED
  drivers/coin_d4_driver/src/lidar_sdk/lidar_data_processor.cpp
  drivers/coin_d4_driver/src/lidar_sdk/mtime.cpp
  drivers/coin_d4_driver/src/lidar_sdk/serial_port.cpp
  drivers/coin_d4_driver/src/lidar_sdk/timer.cpp
  drivers/coin_d4_driver/src/coin_d4_base_handler.cpp
  drivers/coin_d4_driver/src/coin_d4_lifecycle_handler.cpp
  drivers/coin_d4_driver/src/coin_d4_node_handler.cpp
)
target_include_directories(coin_d4_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivers/coin_d4_driver/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(coin_d4_driver_lib
  rclcpp rclcpp_lifecycle sensor_msgs geometry_msgs std_msgs)

add_executable(single_coin_d4_node
  drivers/coin_d4_driver/src/single_coin_d4_node.cpp)
target_link_libraries(single_coin_d4_node coin_d4_driver_lib)

install(TARGETS
  localization_slam
  turtlebot3_ros
  ld08_driver
  single_coin_d4_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  turtlebot3_control_lib
  coin_d4_driver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(shared_dir config launch description param script rviz)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${shared_dir}")
    install(DIRECTORY "${shared_dir}/"
      DESTINATION share/${PROJECT_NAME}/${shared_dir})
  endif()
endforeach()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/drivers/ld08_driver/launch")
  install(DIRECTORY drivers/ld08_driver/launch/
    DESTINATION share/${PROJECT_NAME}/drivers/ld08_driver/launch)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/drivers/coin_d4_driver/launch")
  install(DIRECTORY drivers/coin_d4_driver/launch/
    DESTINATION share/${PROJECT_NAME}/drivers/coin_d4_driver/launch)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/drivers/coin_d4_driver/params")
  install(DIRECTORY drivers/coin_d4_driver/params/
    DESTINATION share/${PROJECT_NAME}/drivers/coin_d4_driver/params)
endif()

install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY drivers/coin_d4_driver/include/
  DESTINATION include)
install(DIRECTORY drivers/ld08_driver/include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(turtlebot3_control_lib coin_d4_driver_lib)
ament_export_dependencies(
  dynamixel_sdk
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  rcutils
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  turtlebot3_msgs
)

ament_package()
