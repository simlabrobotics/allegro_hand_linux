
cmake_minimum_required(VERSION 3.8)
project(control_hand)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS})

Set (AllegroHand_VERSION_MAJOR 1)
Set (AllegroHand_VERSION_MINOR 0)

set (CMAKE_CXX_FLAGS "-O2 -Wall")

add_library(
  control_hand
  src/canAPI.cpp
  src/can_communicator.cpp
  src/RockScissorsPaper.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(controller src/controller.cpp)

ament_target_dependencies(controller rclcpp std_msgs std_srvs)

target_include_directories(control_hand
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
"srv/Move.srv"
LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces
  "rosidl_typesupport_cpp"
)

target_link_libraries(control_hand "${cpp_typesupport_target}")

install(TARGETS control_hand
  EXPORT control_hand-targets
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/control_hand DESTINATION include)

target_link_libraries(controller control_hand BHand pcanbasic pthread)

install(EXPORT control_hand-targets
  FILE control_hand-targets.cmake
  NAMESPACE control_hand::
  DESTINATION lib/cmake/${PROJECT_NAME}) 
  
install(TARGETS controller
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)




ament_package()
