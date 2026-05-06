cmake_minimum_required(VERSION 3.8)
project(mbf_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY ros2_control meshes launch config urdf rviz xacro
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()