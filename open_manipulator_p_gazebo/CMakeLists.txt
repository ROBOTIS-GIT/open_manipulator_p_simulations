################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_p_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "std_msgs"
  "rclcpp"
  "controller_manager"
  "urdf"
  "xacro"
)

set(EXEC_NAME "open_manipulator_p_gazebo")

add_executable(${EXEC_NAME} src/omp_gripper_sub_publisher.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY param launch meshes models urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(controller_manager)
ament_export_dependencies(urdf)
ament_export_dependencies(xacro)
ament_package()
