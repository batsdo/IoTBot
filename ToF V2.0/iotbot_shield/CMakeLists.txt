cmake_minimum_required(VERSION 3.5)
project(iotbot_shield)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(iotbot_interface REQUIRED)
find_package(std_srvs)

set(NODE_NAME iotbot_shield_node)

add_executable(${NODE_NAME}
  src/iotbot_shield.cpp
  src/iotbot_shield_node.cpp)

ament_target_dependencies(${NODE_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  iotbot_interface
  std_srvs)

target_link_libraries(${NODE_NAME} mraa) # additionaly link with mraa library -> has to be installed first via commands in https://github.com/eclipse/mraa

target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()