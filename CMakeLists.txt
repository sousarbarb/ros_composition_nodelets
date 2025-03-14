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

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    nodelet
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    nodelet
    std_msgs
  # DEPENDS
  #   Boost
  #   stage
)

link_directories(${catkin_LIBRARY_DIRS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/pub_ros1.cpp
  src/${PROJECT_NAME}/sub_ros1.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(pub src/main_ros1_pub.cpp)
add_dependencies(pub
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pub ${PROJECT_NAME})

add_executable(sub src/main_ros1_sub.cpp)
add_dependencies(sub
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sub ${PROJECT_NAME})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} pub sub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.launch"
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)





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

install(TARGETS pub2_component sub2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pub2 sub2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.launch.xml"
    PATTERN "*.launch.py"
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
