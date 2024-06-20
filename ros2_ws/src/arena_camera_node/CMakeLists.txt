cmake_minimum_required(VERSION 3.15)

project(arena_camera_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
endif()


# arenasdk find file dir
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# finds dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arena_sdk REQUIRED)
find_package(camera_msg REQUIRED)

# export dependencies

set(target_name start)

add_executable(${target_name}
    src/main.cpp
    src/ArenaCameraNode.cpp
    src/light_arena/deviceinfo_helper.cpp
    src/rclcpp_adapter/pixelformat_translation.cpp
    src/rclcpp_adapter/quilty_of_service_translation.cpp
)

ament_target_dependencies(${target_name}  rclcpp
                                          std_msgs
                                          std_srvs
                                          sensor_msgs
                                          arena_sdk
                                          camera_msg
                                          )

install(TARGETS
  ${target_name}
    DESTINATION lib/${PROJECT_NAME}
)

set(trigger_image trigger_image)

add_executable(${trigger_image} src/trigger_image_client.cpp)

ament_target_dependencies(${trigger_image}  rclcpp
                                          std_srvs)

target_include_directories(${trigger_image}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  ${trigger_image}
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
