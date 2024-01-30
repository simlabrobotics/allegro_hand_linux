
cmake_minimum_required(VERSION 3.8)
project(controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS})

Set (AllegroHand_VERSION_MAJOR 1)
Set (AllegroHand_VERSION_MINOR 0)

set (CMAKE_CXX_FLAGS "-O2 -Wall")

add_library(
controller
src/canAPI.cpp
src/can_communicator.cpp
src/RockScissorsPaper.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(main src/main.cpp) 

target_include_directories(controller
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

install(TARGETS main controller EXPORT controller-targets)

install(DIRECTORY include/controller DESTINATION include)

target_link_libraries (main controller BHand pcanbasic pthread)

install(EXPORT controller-targets
  FILE controller-targets.cmake
  NAMESPACE controller::
  DESTINATION lib/cmake/${PROJECT_NAME})  


ament_package()
