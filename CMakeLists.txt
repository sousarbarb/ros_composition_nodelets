cmake_minimum_required(VERSION 3.5)
project(ros_composition_nodelets)



# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()





##########################################
##### ROBOT OPERATING SYSTEM (ROS) 1 #####
##########################################

if("$ENV{ROS_VERSION}" STREQUAL "1")

message(STATUS "Robot Operating System (ROS) detected: ROS $ENV{ROS_VERSION} $ENV{ROS_DISTRO}")





##########################################
##### ROBOT OPERATING SYSTEM (ROS) 2 #####
##########################################

elseif("$ENV{ROS_VERSION}" STREQUAL "2")

message(STATUS "Robot Operating System (ROS) detected: ROS $ENV{ROS_VERSION} $ENV{ROS_DISTRO}")

## Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

## Declare a C++ executable
add_executable(pub2
  src/ros_composition_nodelets/pub_ros2.cpp
  src/main_ros2_pub.cpp
)
target_include_directories(pub2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pub2
  rclcpp
  std_msgs
)

add_executable(sub2
  src/ros_composition_nodelets/sub_ros2.cpp
  src/main_ros2_sub.cpp
)
target_include_directories(sub2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sub2
  rclcpp
  std_msgs
)

#############
## Install ##
#############

install(
  TARGETS pub2 sub2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

## Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()





## !!!NO ROBOT OPERATING SYSTEM VERSION FOUND!!!

else()

message(FATAL_ERROR "ROS_VERSION environment variable not set...")

endif()
