cmake_minimum_required(VERSION 3.5)
project(alpo_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")

# find ros2 dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_mobile_base REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_mobile_base_utils REQUIRED)
find_package(romea_mobile_base_hardware REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/alpo_hardware.cpp
  src/deadzone_gaussian_noise.cpp
  src/deadzone_dithering.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)

target_compile_definitions(
  ${PROJECT_NAME} 
  PUBLIC 
  ROS_DISTRO=${ROS_DISTRO})

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  romea_core_common
  romea_core_mobile_base
  romea_common_utils
  romea_mobile_base_utils
  romea_mobile_base_hardware
  sensor_msgs
  ackermann_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface alpo_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

##if(BUILD_TESTING)
##  find_package(ament_cmake_gtest REQUIRED)
##endif()

## EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  romea_core_common
  romea_core_mobile_base
  romea_common_utils
  romea_mobile_base_utils
  romea_mobile_base_hardware
  sensor_msgs
  ackermann_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
