cmake_minimum_required(VERSION 3.5)
project(mbf_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
# find_package(champ REQUIRED)
find_package(message_filters REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  controller_manager_msgs
  robot_msgs
  tf2_ros
  urdf
  message_filters
#   champ
)

# 1. Create the library
add_library(mbf_control SHARED src/mbf_control.cpp)
target_include_directories(mbf_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/champ>  # Add this line
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mbf_control ${dependencies})

# 2. Create the Node (links to the library above)
add_executable(mbf_control_node src/mbf_control_node.cpp)
target_link_libraries(mbf_control_node mbf_control)
ament_target_dependencies(mbf_control_node ${dependencies})

# 3. Install headers and libraries
install(DIRECTORY include/ DESTINATION include)

install(TARGETS mbf_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 4. Install executable for ros2 run
install(TARGETS mbf_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mbf_control)
ament_export_dependencies(${dependencies})

ament_package()