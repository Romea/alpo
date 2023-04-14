cmake_minimum_required(VERSION 3.5)

project(alpo_bridge)

if($ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
endif()

if(${ROS_VERSION} EQUAL 1)

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
       set(CMAKE_CXX_STANDARD 14)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
       add_compile_options(-Wall -Wextra)
    endif()

    #find ROS 2 packages
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(ackermann_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)

    #find ROS 1 packages
    set(cmake_extras_files cmake/find_ros1_package.cmake cmake/find_ros1_interface_packages.cmake)
    include(cmake/find_ros1_package.cmake)
    find_ros1_package(roscpp)
    find_ros1_package(std_msgs)
    find_ros1_package(sensor_msgs)
    find_ros1_package(geometry_msgs)
    find_ros1_package(ackermann_msgs)
    find_ros1_package(nav_msgs)

    add_executable(${PROJECT_NAME}
      src/alpo_bridge_node.cpp
      src/alpo_bridge.cpp)

    ament_target_dependencies(${PROJECT_NAME}
      rclcpp
      std_msgs
      sensor_msgs
      ackermann_msgs
      nav_msgs
      ros1_roscpp
      ros1_std_msgs
      ros1_sensor_msgs
      ros1_ackermann_msgs
      ros1_nav_msgs
    )

    install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

    ament_package()

endif()