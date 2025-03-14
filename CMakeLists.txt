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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

## Declare a C++ library
add_library(pub2_component SHARED src/ros_composition_nodelets/pub_ros2.cpp)
target_include_directories(pub2_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(pub2_component PRIVATE
  "ROS_COMPOSITION_NODELETS_BUILDING_DLL"
)
ament_target_dependencies(pub2_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(pub2_component "ros_composition_nodelets::PubROS2")

add_library(sub2_component SHARED src/ros_composition_nodelets/sub_ros2.cpp)
target_include_directories(sub2_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(sub2_component PRIVATE
  "ROS_COMPOSITION_NODELETS_BUILDING_DLL"
)
ament_target_dependencies(sub2_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(sub2_component "ros_composition_nodelets::SubROS2")

## Declare a C++ executable
add_executable(pub2 src/main_ros2_pub.cpp)
target_include_directories(pub2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pub2 pub2_component)

add_executable(sub2 src/main_ros2_sub.cpp)
target_include_directories(sub2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sub2 sub2_component)

#############
## Install ##
#############

install(
  TARGETS pub2_component sub2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
