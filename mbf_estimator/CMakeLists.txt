cmake_minimum_required(VERSION 3.10)
project(mbf_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)

set(common_deps
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(body_estimator
  src/body_estimator.cpp
  src/linear_kf.cpp
)
ament_target_dependencies(body_estimator ${common_deps})
target_include_directories(body_estimator PRIVATE
  include
  
)
target_link_libraries(body_estimator
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS body_estimator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/replay_bag.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_package()
