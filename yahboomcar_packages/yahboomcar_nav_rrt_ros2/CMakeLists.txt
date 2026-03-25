cmake_minimum_required(VERSION 3.8)
project(yahboomcar_nav_rrt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# C++ executables
add_executable(save_map src/save_map.cpp)
ament_target_dependencies(save_map rclcpp std_msgs std_srvs)

add_executable(rrt_save_map src/rrt_save_map.cpp)
ament_target_dependencies(rrt_save_map rclcpp nav_msgs geometry_msgs)

install(TARGETS
  save_map
  rrt_save_map
  DESTINATION lib/${PROJECT_NAME}
)

# Python scripts
install(PROGRAMS
  scripts/scan_dilute.py
  scripts/send_mark.py
  scripts/spech_send_mark.py
  scripts/topic_relay.py
  scripts/tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Config files
install(DIRECTORY
  param
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
